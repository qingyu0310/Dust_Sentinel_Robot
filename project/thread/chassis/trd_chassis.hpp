/**
 * @file trd_chassis.hpp
 * @author qingyu
 * @brief 四轮全向轮底盘定义
 * @version 0.1
 * @date 2026-11-12
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <cstdint>
#include "Irq_handlers.h"
#include "dji_c6xx.hpp"

namespace instance::chassis {

#define CHASSIS_RX  USER_RX_CAN1

constexpr uint8_t  N_Wheel = 4;				// 全向轮数量

// C620 反馈 ID
constexpr uint16_t kDriveCanId[N_Wheel] { 0x201, 0x202, 0x203, 0x204 };

// C620 控制帧 ID
constexpr uint16_t kControlTxId = 0x200;

constexpr float	   kWheelRadius = 0.285f;	// 轮半径 m（570mm 直径）

// 目标 v 比例（遥控归一化 [-1,1] 乘以该值）
inline float chassis_v = 2.0f;

// 小陀螺目标旋转速度
inline float chassis_w = 15.0f;

inline motor::dji::DjiC620 chassis_motor_[N_Wheel] {};

} // namespace instance::chassis
