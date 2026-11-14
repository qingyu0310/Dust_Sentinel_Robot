/**
 * @file trd_pitch.cpp
 * @author qingyu
 * @brief 上 pitch 达妙电机初始化与反馈接入
 * @version 0.2
 * @date 2026-11-14
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <zephyr/zbus/zbus.h>
#include <algorithm>
#include <cmath>
#include "trd_pitch.hpp"
#include "gimbal.hpp"
#include "to_can_tx.hpp"
#include "thread.hpp"
#include "Init_entry.hpp"
#include "Irq_handlers.h"
#include "remote_to.hpp"
#include "angle.hpp"
#include "pid.hpp"
#include "log.hpp"
#include "var.hpp"

namespace thread::pitch {

// 本次起动力矩拟合：g(theta)=0.041619*sin(theta)+0.315195*cos(theta)，theta 为 rad。
// 默认先验证 50%；var 可调至 0 进行对照，静摩擦补偿暂不加入。
static float gravity_scale_ = 1.0f;				// 重力前馈比例，0=关闭
static float target_deg_    = 0.0f;				// 目标角度（度），由 var 设定

/**
 * @brief 重力前馈力矩
 * @param pitch_rad 电机反馈角度，单位 rad
 * @return 补偿力矩，单位 N*m
 */
static float GravityTorque(float pitch_rad)
{
    if (!std::isfinite(pitch_rad) || !std::isfinite(gravity_scale_)) return 0.0f;
    const float scale = std::clamp(gravity_scale_, 0.0f, 1.0f);
    return scale * (0.041619f * std::sin(pitch_rad) + 0.315195f * std::cos(pitch_rad));
}

using namespace instance;
using namespace instance::pitch;
using namespace alg::math;

static Thread<2048> thread_{};

static alg::pid::Pid up_pid_   {};								// 上 pitch 位置环
static gimbal::DtMeter dt_meter_ {};							// 控制周期测量

static topic::remote_to::Message remote_ {};					// 遥控最新状态，读失败保留旧值

static constexpr uint32_t kLostCheckPeriod = 100;				// 掉线检测周期（Task 拍数）

static gimbal::StallWatch remote_watch_ {};						// 遥控停发检测（默认 100 拍）
static gimbal::LostWatch  motor_watch_  { kLostCheckPeriod };	// 电机掉线检测

static bool g_set_zero_up = false;								// 上 pitch 设零请求，执行后自清

/**
 * @brief Retry 钩子：设零与未使能恢复，阻塞等待停稳
 * @param context 未使用
 * @return true=已处理，继续下一拍
 */
static bool OnRetry(void* context)
{
    (void)context;

    if (remote_watch_.Timeout() || remote_.robot_ctrl == topic::remote_to::StartMode::Off)
	{
        uint8_t frame[8];

        DUST_LOG_WRN("pitch disable: stall=%u ctrl=%d ver=%u",
                     static_cast<unsigned>(remote_watch_.Stall()), static_cast<int>(remote_.robot_ctrl),
                     static_cast<unsigned>(remote_.version));
        up_pitch_motor_.PackCmdFrame(frame, DmMotor::Cmd::Disable);
        gimbal::SendCmd(&user_can0_msgq, kUpPitchCanId, frame);

        return true;
    }

    if (g_set_zero_up) {
        g_set_zero_up = false;
        gimbal::SaveZero(up_pitch_motor_, &user_can0_msgq, kUpPitchCanId);
        return true;
    }

    if (!gimbal::Enabled(up_pitch_motor_))
	{
        DUST_LOG_WRN("pitch not enabled: err=%d", static_cast<int>(up_pitch_motor_.ReadAll().err));
        if (gimbal::Enable(up_pitch_motor_, &user_can0_msgq, kUpPitchCanId)) {
            DUST_LOG_INF("pitch re-enabled");
        }
    }

    return true;
}

/**
 * @brief 任务单拍：判断设零标志位和使能状态，异常转 Retry 交 OnRetry 处理
 * @param context 未使用
 * @return Continue=正常；Retry=设零请求或未使能
 */
static TaskResult Task(void* context)
{
    (void)context;

    const float dt = dt_meter_.Update();

    // 直接读话题：每拍取最新状态，不积压历史通知；读失败保留上一帧值
    zbus_chan_read(&pub_remote_to, &remote_, K_NO_WAIT);

    // 输入停更或整车关闭：处置交给 OnRetry
    if (remote_watch_.Update(remote_.version) || remote_.robot_ctrl == topic::remote_to::StartMode::Off) {
        return TaskResult::Retry;
    }

    if (g_set_zero_up || !gimbal::Enabled(up_pitch_motor_)) {
        return TaskResult::Retry;
    }

    // 窗口滑窗检测掉线：flag_ 与 pre_flag_ 相等即本拍无新反馈，按周期分频检查

    if (motor_watch_.Update(up_pitch_motor_)) {
        DUST_LOG_ERR("pitch lost");
        return TaskResult::Retry;
    }

    uint8_t frame[8];

	auto snap = up_pitch_motor_.ReadAll();

	const float up_now_rad	   = snap.radian;
    // 遥控 [-1,1] 线性映射到机械限位，范围已由 remote 处理
    const float up_target_deg  = kUpPitchMinDeg + (kUpPitchMaxDeg - kUpPitchMinDeg) * (remote_.pitch + 1.0f) * 0.5f;
    const float up_target_rad  = up_target_deg * kDegToRad;
    const float up_err 	   	   = WrapRad(up_now_rad - up_target_rad);

    const float pid_torque 	   = up_pid_.Calc(0.0f, up_err, dt);
    const float gravity_torque = GravityTorque(up_now_rad);
    const float up_torque 	   = std::clamp(pid_torque + gravity_torque, -kPitchTorqueMax, kPitchTorqueMax);

    up_pitch_motor_.SetTargetTorque(up_torque);
    up_pitch_motor_.PackCtrlFrame(frame);
    gimbal::SendCmd(&user_can0_msgq, kUpPitchCanId, frame);

    DUST_LOG_DBG("pitch_track", "%.2f,%.2f,%.3f", static_cast<double>(up_target_deg), static_cast<double>(snap.angle), static_cast<double>(snap.torque));

    return TaskResult::Continue;
}

static bool thread_init()
{
    DmMotor::Config cfg{};
    cfg.ctrl_met = ControlMethon::Mit;
    cfg.gearbox_ratio = 10.0f;  			// J4310-2EC 内置减速箱 10:1
    cfg.wheel_r = 0.0f;         			// pitch 是旋转关节，无线速度换算

    cfg.kp = 0.0f;
    cfg.kd = 0.0f;

    cfg.PMAX = 12.5f;
    cfg.VMAX = 30.0f;
    cfg.TMAX = 10.0f;

    cfg.can_id    = kUpPitchCanId;
    cfg.master_id = kUpPitchMasterId;
    up_pitch_motor_.Init(cfg);

    // 上 pitch 位置环：输出力矩
    alg::pid::Pid::Config pid_cfg {};

    pid_cfg.kp     = 6.f;
    pid_cfg.ki     = 0.0f;
    pid_cfg.kd     = 0.085f;
    pid_cfg.outMax = kPitchTorqueMax;

    up_pid_.Init(pid_cfg);

    DUST_LOG_INF("pitch ready: up 0x04/0x14, CAN0");

    return true;
}

/**
 * @brief 启动 pitch 线程
 * @return true=成功
 */
static bool thread_start()
{
    thread_.Start(Task, K_MSEC(1), ThreadPrio::High, nullptr, "pitch", ThreadFaultConfig{ OnRetry, nullptr, ThreadFaultPolicy::Isolate });
    return true;
}

CAN_RX_HANDLER(USER_RX_CAN0, kUpPitchMasterId,   [](uint8_t* data) { up_pitch_motor_  .CanCpltRxCallback(data); });

REGISTER_SHELL_VAR("pitch_zero_up",   g_set_zero_up);
REGISTER_SHELL_VAR("pitch_up_gscale", gravity_scale_);
REGISTER_SHELL_VAR("pitch_up_target", target_deg_);

REGISTER_SHELL_VAR("pitch_up_kp",     up_pid_.GetKp());
REGISTER_SHELL_VAR("pitch_up_ki",     up_pid_.GetKi());
REGISTER_SHELL_VAR("pitch_up_kd",     up_pid_.GetKd());

REGISTER_INIT(thread_init, MidInit, High, HaltOnFail, nullptr, "pitch_init");
REGISTER_THREAD(thread_start, MidThread, ContinueOnFail, "pitch_start");

} // namespace thread::pitch
