/**
 * @file trd_pitch.hpp
 * @author qingyu
 * @brief 上 pitch 达妙电机定义
 * @version 0.2
 * @date 2026-11-14
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <cstdint>
#include "dm.hpp"

namespace instance::pitch {

// 上 pitch 接 CAN0；反馈帧使用 Master ID。
constexpr uint16_t kUpPitchCanId      = 0x04;
constexpr uint16_t kUpPitchMasterId   = 0x14;

constexpr float    kPitchTorqueMax    = 3.0f;		// pitch 峰值力矩 N*m

constexpr float    kUpPitchMinDeg     = -10.0f;		// 上 pitch 机械下限（度）
constexpr float    kUpPitchMaxDeg     =  40.0f;		// 上 pitch 机械上限（度）

inline DmMotor up_pitch_motor_  {};

} // namespace instance::pitch
