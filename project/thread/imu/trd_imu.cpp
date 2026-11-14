/**
 * @file trd_imu.cpp
 * @author qingyu
 * @brief IMU 线程 — 数据采集、加热控温、姿态解算
 * @version 0.1
 * @date 2026-11-04
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma message "Compiling Thread/Imu"

#include "imu.hpp"
#include "Init_entry.hpp"

namespace thread::imu {

static ::imu::ImuManager imu_ {};

// 本颗 IMU 的静态零偏（gyro rad/s，accel m/s^2）
static constexpr ImuOffsetData kOffset {
    {0.009049f,  0.011271f, -0.009992f },
    {-0.420728f, 0.042918f, 1.061736f }
};

bool thread_init()
{
    return imu_.Init(::imu::ImuStartMode::AutoCalib, kOffset);
}

bool thread_start()
{
    return imu_.Start(ThreadPrio::High);
}

REGISTER_INIT(thread_init, EarlyInit, High, HaltOnFail, nullptr, "imu_init");
REGISTER_THREAD(thread_start, EarlyThread, HaltOnFail, "imu_start");

} // namespace thread::imu
