/**
 * @file trd_yaw.cpp
 * @author qingyu
 * @brief 双 yaw 达妙电机初始化与反馈接入
 * @version 0.2
 * @date 2026-11-13
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <zephyr/zbus/zbus.h>
#include "trd_yaw.hpp"
#include "trd_yaw_cmd.hpp"
#include "trd_yaw_coop.hpp"
#include "thread.hpp"
#include "Init_entry.hpp"
#include "Irq_handlers.h"
#include "remote_to.hpp"
#include "imu_to.hpp"
#include "angle.hpp"
#include "pid.hpp"
#include "log.hpp"
#include "var.hpp"

namespace thread::yaw {

using namespace instance::yaw;
using namespace alg::math;

static Thread<1024> thread_{};

static Coop coop_ {};								// 大小 yaw 世界角分配
static bool coop_ready_ = false;					// 已用当前姿态初始化

static alg::pid::Pid big_pid_   {};					// 大 yaw 关节角环
static alg::pid::Pid small_pid_ {};					// 小 yaw 关节角环

static bool g_set_zero_big   = false;				// 大 yaw 设零请求，执行后自清
static bool g_set_zero_small = false;				// 小 yaw 设零请求，执行后自清

static constexpr uint32_t kImuTimeoutMs    = 20;	// IMU 话题超时，超过即判停更
static constexpr uint32_t kRemoteTimeoutMs = 100;	// 遥控话题超时，超过即判失联

static topic::remote_to::Message remote_    {};		// 遥控最新状态，读失败保留旧值
static topic::imu_to::Message    imu_msg_   {};		// IMU 最新状态，读失败保留旧值
static uint32_t                  remote_ms_ = 0;	// 遥控最近一次读取成功时刻
static uint32_t                  imu_ms_    = 0;	// IMU 最近一次读取成功时刻

/**
 * @brief Retry 钩子：设零与未使能恢复，阻塞等待停稳
 * @param context 未使用
 * @return true=已处理，继续下一拍
 */
static bool OnRetry(void* context)
{
    (void)context;

    const uint32_t now = k_uptime_get_32();

    const bool remote_alive = (remote_ms_ != 0) && (now - remote_ms_ < kRemoteTimeoutMs);
    const bool imu_alive    = (imu_ms_    != 0) && (now - imu_ms_    < kImuTimeoutMs);

    if (!remote_alive || remote_.robot_ctrl == topic::remote_to::StartMode::Off) 
	{
        uint8_t frame[8];

        big_yaw_motor_.PackCmdFrame(frame, DmMotor::Cmd::Disable);
        SendCmd(kBigYawCanId, frame);

        k_msleep(kCanGapMs);

        small_yaw_motor_.PackCmdFrame(frame, DmMotor::Cmd::Disable);
        SendCmd(kSmallYawCanId, frame);

        coop_ready_ = false;		// 重新开启时用当前姿态重建参考
        return true;
    }

    if (!imu_alive) {
        uint8_t frame[8];

        big_yaw_motor_.SetTargetTorque(0.0f);
        big_yaw_motor_.PackCtrlFrame(frame);
        SendCmd(kBigYawCanId, frame);

        k_msleep(kCanGapMs);

        small_yaw_motor_.SetTargetTorque(0.0f);
        small_yaw_motor_.PackCtrlFrame(frame);
        SendCmd(kSmallYawCanId, frame);

        return true;
    }

    if (g_set_zero_big)
	{
        g_set_zero_big = false;
        SaveZero(true);
        return true;
    }

    if (g_set_zero_small) 
	{
        g_set_zero_small = false;
        SaveZero(false);
        return true;
    }

    if (!BothEnabled())
	{
        if (EnableBoth()) {
            DUST_LOG_INF("yaw re-enabled");
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

    // 直接读话题：每拍取最新状态，不积压历史通知；读失败保留上一帧值
    const uint32_t now = k_uptime_get_32();

    if (zbus_chan_read(&pub_remote_to, &remote_, K_NO_WAIT) == 0) {
        remote_ms_ = now;
    }

    if (zbus_chan_read(&pub_imu_to,    &imu_msg_, K_NO_WAIT) == 0) {
        imu_ms_ = now;
    }

    // 输入超时或整车关闭：处置交给 OnRetry
    const bool remote_alive = (remote_ms_ != 0) && (now - remote_ms_ < kRemoteTimeoutMs);
    const bool imu_alive    = (imu_ms_    != 0) && (now - imu_ms_    < kImuTimeoutMs);

    if (!remote_alive || remote_.robot_ctrl == topic::remote_to::StartMode::Off || !imu_alive) {
        return TaskResult::Retry;
    }

    if (g_set_zero_big || g_set_zero_small || !BothEnabled()) {
        return TaskResult::Retry;
    }

    const DmMotor::Snapshot big   = big_yaw_motor_  .ReadAll();
    const DmMotor::Snapshot small = small_yaw_motor_.ReadAll();

    // 达妙反馈已是输出轴角度与角速度；大 yaw 逆时针为负，取反统一成逆时针为正
    // 多圈累计值归一化到 [-π, π]
    const float big_joint   = WrapRad(-big.radian);
    const float small_joint = WrapRad(small.radian);
    const float world_yaw   = WrapRad(imu_msg_.yaw * kDegToRad); // IMU 发布单位为度，转为 rad

    // 首拍用当前姿态建立协同参考
    if (!coop_ready_) {
        coop_.Init(big_joint, world_yaw);
        coop_ready_ = true;
    }

    // 世界角分配：得到两轴目标角
    float big_target   = 0.0f;
    float small_target = 0.0f;
    float world_target = 0.0f;

    coop_.Update(remote_.yaw, world_yaw, big_joint, small_joint, kControlDT, big_target, small_target, world_target);

    // 大轴闭环关节角误差；小轴闭环世界角误差（与参考实现一致），PID 目标恒为 0
    const float big_err   = WrapRad(big_joint   - big_target);
    const float small_err = WrapRad(world_yaw   - world_target);

    const float big_torque   = big_pid_  .Calc(0.0f, big_err);
    const float small_torque = small_pid_.Calc(0.0f, small_err);

    // 大 yaw 物理方向相反，输出取反
    uint8_t frame[8];

    big_yaw_motor_.SetTargetTorque(-big_torque);
    big_yaw_motor_.PackCtrlFrame(frame);
    SendCmd(kBigYawCanId, frame);

    small_yaw_motor_.SetTargetTorque(small_torque);
    small_yaw_motor_.PackCtrlFrame(frame);
    SendCmd(kSmallYawCanId, frame);

    return TaskResult::Continue;
}

static bool thread_init()
{
    DmMotor::Config cfg{};
    cfg.ctrl_met = ControlMethon::Mit;
    cfg.gearbox_ratio = kGearRatio;			// J4310-2EC 内置减速箱 10:1，同步带 1:1
    cfg.wheel_r = 0.0f;         			// yaw 是旋转关节，无线速度换算

    cfg.kp = 0.0f;
    cfg.kd = 0.0f;

    // 暂按上位机截图配置，两台电机的实际读回量程需与此一致。
    cfg.PMAX = 12.5f;
    cfg.VMAX = 30.0f;
    cfg.TMAX = 10.0f;

    cfg.can_id    = kBigYawCanId;
    cfg.master_id = kBigYawMasterId;
    big_yaw_motor_.Init(cfg);

    cfg.can_id	  = kSmallYawCanId;
    cfg.master_id = kSmallYawMasterId;
    small_yaw_motor_.Init(cfg);

    // 两轴 PID：位置环输出力矩
    alg::pid::Pid::Config pid_cfg {};

    pid_cfg.kp     = 1.0f;
    pid_cfg.ki     = 0.0f;
    pid_cfg.kd     = 0.0f;
    pid_cfg.outMax = kYawTorqueMax;

    big_pid_.Init(pid_cfg);
    small_pid_.Init(pid_cfg);

    DUST_LOG_INF("yaw ready: big CAN1 0x01/0x11, small CAN0 0x03/0x13");

    return true;
}

/**
 * @brief 启动 yaw 线程
 * @return true=成功
 */
static bool thread_start()
{
    thread_.Start(Task, K_MSEC(1), ThreadPrio::High, nullptr, "yaw", ThreadFaultConfig{ OnRetry, nullptr, ThreadFaultPolicy::Isolate });
    return true;
}

CAN_RX_HANDLER(USER_RX_CAN0, kSmallYawMasterId, [](uint8_t* data) { small_yaw_motor_.CanCpltRxCallback(data); });
CAN_RX_HANDLER(USER_RX_CAN1, kBigYawMasterId,   [](uint8_t* data) { big_yaw_motor_  .CanCpltRxCallback(data); });

REGISTER_SHELL_VAR("yaw_zero_big",   g_set_zero_big);
REGISTER_SHELL_VAR("yaw_zero_small", g_set_zero_small);

REGISTER_INIT(thread_init, MidInit, High, HaltOnFail, nullptr, "yaw_init");
REGISTER_THREAD(thread_start, MidThread, ContinueOnFail, "yaw_start");

} // namespace thread::yaw
