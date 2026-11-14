/**
 * @file trd_test.cpp
 * @author qingyu
 * @brief IMU 姿态诊断：高速晃动与机械限位下的漂移观测
 * @version 0.1
 * @date 2026-11-14
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <cmath>
#include <zephyr/zbus/zbus.h>
#include "thread.hpp"
#include "Init_entry.hpp"
#include "imu_to.hpp"
#include "log.hpp"
#include "var.hpp"

namespace thread::test {

static Thread<1024> thread_ {};

static int32_t  on_      = 0;			// 采集中：1=打印
static int32_t  divisor_ = 10;			// 打印分频（每 N 拍一条）
static uint32_t count_   = 0;			// 已过拍数

static topic::imu_to::Message imu_ {};	// IMU 最新状态

/**
 * @brief 任务单拍：按分频把 IMU 原始量与解算姿态打成一条
 * @param context 未使用
 * @return Continue
 */
static TaskResult Task(void* context)
{
    (void)context;

    zbus_chan_read(&pub_imu_to, &imu_, K_NO_WAIT);

    if (on_ == 0) return TaskResult::Continue;

    if (++count_ < static_cast<uint32_t>(divisor_ < 1 ? 1 : divisor_)) {
        return TaskResult::Continue;
    }
    count_ = 0;

    // 加速度模长：静止时约 9.8，晃动时运动加速度混入会偏离，EKF 会拿它修正姿态
    const float acc_norm = std::sqrt(imu_.accel[0] * imu_.accel[0] + imu_.accel[1] * imu_.accel[1] + imu_.accel[2] * imu_.accel[2]);

    DUST_LOG_INF("IMUDBG,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                 static_cast<int>(imu_.roll     * 1000),
                 static_cast<int>(imu_.pitch    * 1000),
                 static_cast<int>(imu_.yaw      * 1000),
                 static_cast<int>(imu_.gyro[0]  * 1000),
                 static_cast<int>(imu_.gyro[1]  * 1000),
                 static_cast<int>(imu_.gyro[2]  * 1000),
                 static_cast<int>(imu_.accel[0] * 1000),
                 static_cast<int>(imu_.accel[1] * 1000),
                 static_cast<int>(imu_.accel[2] * 1000),
                 static_cast<int>(acc_norm * 1000),
                 static_cast<int>(imu_.temperature * 1000));

    return TaskResult::Continue;
}

static bool thread_init()
{
    DUST_LOG_INF("imu diag ready");
    return true;
}

/**
 * @brief 启动 IMU 诊断线程
 * @return true=成功
 */
static bool thread_start()
{
    thread_.Start(Task, K_MSEC(1), ThreadPrio::Normal, nullptr, "test", ThreadFaultConfig{ nullptr, nullptr, ThreadFaultPolicy::Isolate });
    return true;
}

REGISTER_SHELL_VAR("imu_diag_on",   on_);
REGISTER_SHELL_VAR("imu_diag_div",  divisor_);

REGISTER_INIT(thread_init, MidInit, High, HaltOnFail, nullptr, "test_init");
REGISTER_THREAD(thread_start, MidThread, ContinueOnFail, "test_start");

} // namespace thread::test
