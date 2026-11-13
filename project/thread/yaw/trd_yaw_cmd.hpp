/**
 * @file trd_yaw_cmd.hpp
 * @author qingyu
 * @brief yaw 达妙电机命令处理：使能、清错、设零
 * @version 0.1
 * @date 2026-11-13
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <cstdint>

namespace instance::yaw {

/**
 * @brief 发一帧 DM 命令帧（使能/失能等）
 * @param can_id  电机控制帧 ID
 * @param data    8 字节命令帧
 */
void SendCmd(uint16_t can_id, const uint8_t (&data)[8]);

/**
 * @brief 使能两台 yaw：清错误 → 等 1s → 读状态确认 → 使能 → 等 1s
 * @return true=已使能；false=清错未生效或使能失败
 */
bool EnableBoth();

/**
 * @brief 下发设零并等 1s 停稳
 * @param big  true=大 yaw，false=小 yaw
 */
void SaveZero(bool big);

/**
 * @brief 两台 yaw 是否都处于使能态
 * @return true=都已使能
 */
bool BothEnabled();

} // namespace instance::yaw
