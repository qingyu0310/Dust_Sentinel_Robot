/**
 * @file trd_test.hpp
 * @author qingyu
 * @brief 电机阶跃辨识测试线程定义
 * @version 0.2
 * @date 2026-11-13
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <cstdint>
#include "Irq_handlers.h"

namespace instance::test {

#define TEST_RX  USER_RX_CAN1

constexpr uint8_t N_Motor = 4;

// 辨识用电机反馈 ID（与 chassis 四个轮一致）
constexpr uint16_t kTestCanId[N_Motor] = { 0x201, 0x202, 0x203, 0x204 };

// C620 控制帧 ID
constexpr uint16_t kControlTxId = 0x200;

} // namespace instance::test
