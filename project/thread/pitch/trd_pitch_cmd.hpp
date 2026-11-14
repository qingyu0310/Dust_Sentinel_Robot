/**
 * @file trd_pitch_cmd.hpp
 * @author qingyu
 * @brief 上 pitch 达妙电机命令处理：使能、清错、设零
 * @version 0.2
 * @date 2026-11-14
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <cstdint>

namespace instance::pitch {

/**
 * @brief 发一帧 DM 命令帧（使能/失能等）
 * @param can_id  电机控制帧 ID
 * @param data    8 字节命令帧
 */
void SendCmd(uint16_t can_id, const uint8_t (&data)[8]);

/**
 * @brief 使能上 pitch：清错误 → 等 1s → 读状态确认 → 使能 → 等 1s
 * @return true=已使能；false=清错未生效或使能失败
 */
bool Enable();

/**
 * @brief 下发设零并等 1s 停稳
 */
void SaveZero();

/**
 * @brief 上 pitch 是否处于使能态
 * @return true=已使能
 */
bool Enabled();

} // namespace instance::pitch
