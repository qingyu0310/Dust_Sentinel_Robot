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
#include "trd_pitch_cmd.hpp"
#include "thread.hpp"
#include "Init_entry.hpp"
#include "Irq_handlers.h"
#include "remote_to.hpp"
#include "angle.hpp"
#include "pid.hpp"
#include "timer.hpp"
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

using namespace instance::pitch;
using namespace alg::math;

static Thread<2048> thread_{};

static alg::pid::Pid up_pid_   {};							// 上 pitch 位置环

static constexpr uint32_t kLostCheckPeriod = 100;			// 掉线检测周期（Task 拍数）
static Timer lost_timer_ { kLostCheckPeriod };		// 掉线检测分频

static bool g_set_zero_up = false;							// 上 pitch 设零请求，执行后自清

static constexpr uint32_t kRemoteTimeoutMs = 100;			// 遥控话题超时，超过即判失联

static topic::remote_to::Message remote_    {};				// 遥控最新状态，读失败保留旧值
static uint32_t                  remote_ms_ = 0;			// 遥控最近一次读取成功时刻

/**
 * @brief Retry 钩子：设零与未使能恢复，阻塞等待停稳
 * @param context 未使用
 * @return true=已处理，继续下一拍
 */
static bool OnRetry(void* context)
{
    (void)context;

    const uint32_t now = k_cyc_to_ms_floor32(k_cycle_get_32());

    const bool remote_alive = (remote_ms_ != 0) && (now - remote_ms_ < kRemoteTimeoutMs);

    if (!remote_alive || remote_.robot_ctrl == topic::remote_to::StartMode::Off)
	{
        uint8_t frame[8];

        DUST_LOG_WRN("pitch disable: alive=%d ctrl=%d now=%u last=%u gap=%u",
                     static_cast<int>(remote_alive), static_cast<int>(remote_.robot_ctrl),
                     static_cast<unsigned>(now), static_cast<unsigned>(remote_ms_),
                     static_cast<unsigned>(now - remote_ms_));
        up_pitch_motor_.PackCmdFrame(frame, DmMotor::Cmd::Disable);
        SendCmd(kUpPitchCanId, frame);

        return true;
    }

    if (g_set_zero_up) {
        g_set_zero_up = false;
        SaveZero();
        return true;
    }

    if (!Enabled())
	{
        DUST_LOG_WRN("pitch not enabled: err=%d", static_cast<int>(up_pitch_motor_.ReadAll().err));
        if (Enable()) {
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

    // 本拍时基只取一次：cycle 算控制周期，同一时刻换算成毫秒做话题超时判断
    float dt = 0.001f;

    const auto kMeasureClock = [&]()
    {
        static uint32_t last_cycle = 0;
        const  uint32_t now_cycle  = k_cycle_get_32();

        // 用真实经过时间算控制周期，避免 D 项按固定 1ms 计算而虚高数倍
        dt = 0.001f;
        if (last_cycle != 0) 
		{
            const uint64_t delta_ns = k_cyc_to_ns_floor64(now_cycle - last_cycle);
            if (delta_ns != 0) {
                // Retry 路径会阻塞 1s，钳位避免单拍积分突跳
                dt = std::clamp(static_cast<float>(delta_ns) * 1.0e-9f, 0.0001f, 0.01f);
            }
        }
        last_cycle = now_cycle;

        return k_cyc_to_ms_floor32(now_cycle);
    };

    const uint32_t now = kMeasureClock();

    // 直接读话题：每拍取最新状态，不积压历史通知；读失败保留上一帧值

    if (zbus_chan_read(&pub_remote_to, &remote_, K_NO_WAIT) == 0) {
        remote_ms_ = now;
    }

    // 输入超时或整车关闭：处置交给 OnRetry
    const bool remote_alive = (remote_ms_ != 0) && (now - remote_ms_ < kRemoteTimeoutMs);

    if (!remote_alive || remote_.robot_ctrl == topic::remote_to::StartMode::Off) {
        return TaskResult::Retry;
    }

    if (g_set_zero_up || !Enabled()) {
        return TaskResult::Retry;
    }

    // 窗口滑窗检测掉线：flag_ 与 pre_flag_ 相等即本拍无新反馈，按周期分频检查

    lost_timer_.Update();
    if (lost_timer_.IsFinish())
	{
        const bool up_lost = up_pitch_motor_.PwrLossCheck();

        if (up_lost) {
            DUST_LOG_ERR("pitch lost: up=%d", static_cast<int>(up_lost));
            return TaskResult::Retry;
        }
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
    SendCmd(kUpPitchCanId, frame);

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
