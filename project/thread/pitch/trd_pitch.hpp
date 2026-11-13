/**
 * @file trd_pitch.hpp
 * @author qingyu
 * @brief 双 pitch 达妙电机定义
 * @version 0.1
 * @date 2026-11-13
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <cstdint>
#include "dm.hpp"

namespace instance::pitch {

// 两台 pitch 都接 CAN0；反馈帧使用 Master ID。
constexpr uint16_t kUpPitchCanId = 0x04;
constexpr uint16_t kUpPitchMasterId = 0x14;
constexpr uint16_t kDownPitchCanId = 0x02;
constexpr uint16_t kDownPitchMasterId = 0x12;

inline DmMotor up_pitch_motor_{};
inline DmMotor down_pitch_motor_{};

} // namespace instance::pitch
