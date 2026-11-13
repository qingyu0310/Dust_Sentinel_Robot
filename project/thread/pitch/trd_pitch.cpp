/**
 * @file trd_pitch.cpp
 * @author qingyu
 * @brief 双 pitch 达妙电机初始化与反馈接入
 * @version 0.1
 * @date 2026-11-13
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "trd_pitch.hpp"
#include "trd_pitch_cmd.hpp"
#include "thread.hpp"
#include "Init_entry.hpp"
#include "Irq_handlers.h"
#include "log.hpp"
#include "var.hpp"

namespace thread::pitch {

using namespace instance::pitch;

static Thread<> thread_{};

static bool g_set_zero_up   = false;	// 上 pitch 设零请求，执行后自清
static bool g_set_zero_down = false;	// 下 pitch 设零请求，执行后自清

/**
 * @brief Retry 钩子：设零与未使能恢复，阻塞等待停稳
 * @param context 未使用
 * @return true=已处理，继续下一拍
 */
static bool OnRetry(void* context)
{
    (void)context;

    if (g_set_zero_up) {
        g_set_zero_up = false;
        SaveZero(true);
        return true;
    }

    if (g_set_zero_down) {
        g_set_zero_down = false;
        SaveZero(false);
        return true;
    }

    if (!BothEnabled()) {
        if (EnableBoth()) {
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

    if (g_set_zero_up || g_set_zero_down || !BothEnabled()) {
        return TaskResult::Retry;
    }

    const DmMotor::Snapshot up   = up_pitch_motor_  .ReadAll();
    const DmMotor::Snapshot down = down_pitch_motor_.ReadAll();

    DUST_LOG_DBG("pitch_angle", "%.2f,%.2f,%.2f,%.2f", static_cast<double>(up.angle), static_cast<double>(up.omega),
															   static_cast<double>(down.angle), static_cast<double>(down.omega));

    return TaskResult::Continue;
}

static bool thread_init()
{
    DmMotor::Config cfg{};
    cfg.ctrl_met = ControlMethon::Mit;
    cfg.gearbox_ratio = 10.0f;  	// J4310-2EC 内置减速箱 10:1
    cfg.wheel_r = 0.0f;         	// pitch 是旋转关节，无线速度换算

    cfg.kp = 0.0f;
    cfg.kd = 0.0f;

    cfg.PMAX = 12.5f;
    cfg.VMAX = 30.0f;
    cfg.TMAX = 10.0f;

    cfg.can_id    = kUpPitchCanId;
    cfg.master_id = kUpPitchMasterId;
    up_pitch_motor_.Init(cfg);

    cfg.can_id    = kDownPitchCanId;
    cfg.master_id = kDownPitchMasterId;
    down_pitch_motor_.Init(cfg);

    // 先使能一次；失败只告警，由 Task 的 Retry 流程继续重试
    if (!EnableBoth()) {
        DUST_LOG_WRN("pitch enable failed at init, will retry in thread");
    }

    DUST_LOG_INF("pitch ready: up 0x04/0x14, down 0x02/0x12, both CAN0");

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
CAN_RX_HANDLER(USER_RX_CAN0, kDownPitchMasterId, [](uint8_t* data) { down_pitch_motor_.CanCpltRxCallback(data); });

REGISTER_SHELL_VAR("pitch_zero_up", g_set_zero_up);
REGISTER_SHELL_VAR("pitch_zero_down", g_set_zero_down);

REGISTER_INIT(thread_init, MidInit, High, HaltOnFail, nullptr, "pitch_init");
REGISTER_THREAD(thread_start, MidThread, ContinueOnFail, "pitch_start");

} // namespace thread::pitch
