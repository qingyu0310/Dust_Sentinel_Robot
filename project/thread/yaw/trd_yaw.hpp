/**
 * @file trd_yaw.hpp
 * @author qingyu
 * @brief 双 yaw 达妙电机定义
 * @version 0.1
 * @date 2026-11-13
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <cstdint>
#include "dm.hpp"

namespace instance::yaw {

// 大 yaw 接 CAN1，小 yaw 接 CAN0；反馈帧使用 Master ID。
constexpr uint16_t kBigYawCanId = 0x01;
constexpr uint16_t kBigYawMasterId = 0x11;
constexpr uint16_t kSmallYawCanId = 0x03;
constexpr uint16_t kSmallYawMasterId = 0x13;

inline DmMotor big_yaw_motor_{};
inline DmMotor small_yaw_motor_{};

} // namespace instance::yaw
