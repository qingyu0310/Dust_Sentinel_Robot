/**
 * @file trd_yaw.hpp
 * @author qingyu
 * @brief 大 yaw 达妙电机定义
 * @version 0.2
 * @date 2026-11-14
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <cstdint>
#include "dm.hpp"

namespace instance::yaw {

// 大 yaw 接 CAN1；反馈帧使用 Master ID。
constexpr uint16_t kBigYawCanId 	 = 0x01;
constexpr uint16_t kBigYawMasterId 	 = 0x11;

constexpr float	   kGearRatio = 10.0f;		// J4310-2EC 内置减速箱 10:1，同步带 1:1
constexpr float    kControlDT = 0.001f;		// 控制周期 s（Task 1ms）

constexpr uint32_t kCanGapMs = 1000;		// 电机命令之间的间隔

inline 	  DmMotor  big_yaw_motor_{};

} // namespace instance::yaw
