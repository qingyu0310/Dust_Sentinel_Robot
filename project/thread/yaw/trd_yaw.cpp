/**
 * @file trd_yaw.cpp
 * @author qingyu
 * @brief 双 yaw 达妙电机初始化与反馈接入
 * @version 0.1
 * @date 2026-11-13
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "trd_yaw.hpp"
#include "trd_yaw_cmd.hpp"
#include "thread.hpp"
#include "Init_entry.hpp"
#include "Irq_handlers.h"
#include "log.hpp"
#include "var.hpp"

namespace thread::yaw {

using namespace instance::yaw;

static Thread<1024> thread_{};

static bool g_set_zero_big   = false;	// 大 yaw 设零请求，执行后自清
static bool g_set_zero_small = false;	// 小 yaw 设零请求，执行后自清

/**
 * @brief Retry 钩子：设零与未使能恢复，阻塞等待停稳
 * @param context 未使用
 * @return true=已处理，继续下一拍
 */
static bool OnRetry(void* context)
{
    (void)context;

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

    if (g_set_zero_big || g_set_zero_small || !BothEnabled()) {
        return TaskResult::Retry;
    }

    // 发空力矩帧（MIT: kp=0, kd=0, t_ff=0），保持电机使能
    {
        uint8_t frame[8];

        big_yaw_motor_.SetTargetTorque(0.0f);
        big_yaw_motor_.PackCtrlFrame(frame);
        SendCmd(kBigYawCanId, frame);

        small_yaw_motor_.SetTargetTorque(0.0f);
        small_yaw_motor_.PackCtrlFrame(frame);
        SendCmd(kSmallYawCanId, frame);
    }

    const DmMotor::Snapshot big   = big_yaw_motor_  .ReadAll();
    const DmMotor::Snapshot small = small_yaw_motor_.ReadAll();

    DUST_LOG_DBG("yaw_angle", "%.2f,%.2f,%.2f,%.2f", static_cast<double>(big.angle), static_cast<double>(big.omega),
															   static_cast<double>(small.angle), static_cast<double>(small.omega));

    return TaskResult::Continue;
}

static bool thread_init()
{
    DmMotor::Config cfg{};
    cfg.ctrl_met = ControlMethon::Mit;
    cfg.gearbox_ratio = 10.0f;  			// J4310-2EC 内置减速箱 10:1，同步带 1:1，总减速比 10
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

    // 先使能一次；失败只告警，由 Task 的 Retry 流程继续重试
    if (!EnableBoth()) {
        DUST_LOG_WRN("yaw enable failed at init, will retry in thread");
    }

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

REGISTER_SHELL_VAR("yaw_zero_big", g_set_zero_big);
REGISTER_SHELL_VAR("yaw_zero_small", g_set_zero_small);

REGISTER_INIT(thread_init, MidInit, High, HaltOnFail, nullptr, "yaw_init");
REGISTER_THREAD(thread_start, MidThread, ContinueOnFail, "yaw_start");

} // namespace thread::yaw
