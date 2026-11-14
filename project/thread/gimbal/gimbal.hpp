/**
 * @file gimbal.hpp
 * @author qingyu
 * @brief 云台达妙电机公共命令与控制周期测量
 * @version 0.2
 * @date 2026-11-14
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <cstdint>
#include <algorithm>
#include <zephyr/kernel.h>
#include "dm.hpp"

namespace instance::gimbal {

/**
 * @brief 控制周期测量：相邻两拍 cycle 差值换算成秒
 *
 * cycle 只用于求差，不回绕成绝对时间；Retry 路径会阻塞 1s，结果钳位避免积分突跳。
 */
class DtMeter
{
public:
    /**
     * @brief 本拍控制周期
     * @return 周期秒数，首拍或异常时返回 1ms
     */
    float Update()
    {
        const uint32_t now_cycle = k_cycle_get_32();
        float dt = 0.001f;

        if (last_cycle_ != 0) {
            const uint64_t delta_ns = k_cyc_to_ns_floor64(now_cycle - last_cycle_);
            if (delta_ns != 0) {
                dt = std::clamp(static_cast<float>(delta_ns) * 1.0e-9f, 0.0001f, 0.01f);
            }
        }
        last_cycle_ = now_cycle;

        return dt;
    }

private:
    uint32_t last_cycle_ = 0;	// 上一拍 cycle 计数
};

/**
 * @brief 帧序号停更检测：序号连续不变即判对端停发
 *
 * 调用方每拍把话题里最新帧的 version 传进来，内部统计连续无新帧拍数。
 */
class StallWatch
{
public:
    /**
     * @brief 更新一拍的帧序号
     * @param version 本拍读到的最新帧序号
     * @return true=已停更达到上限
     */
    bool Update(uint32_t version)
    {
        if (version == seen_) {
            ++stall_;
        } else {
            seen_  = version;
            stall_ = 0;
        }

        return stall_ >= limit_;
    }

    /**
     * @brief 停更拍数上限
     * @param limit 连续无新帧拍数达到该值即判停更
     */
    void SetLimit(uint32_t limit) { limit_ = limit; }

    /**
     * @brief 是否已停更，不改变内部计数
     * @return true=当前已判停更
     */
    bool Timeout() const { return stall_ >= limit_; }

    /**
     * @brief 当前连续无新帧拍数
     * @return 拍数
     */
    uint32_t Stall() const { return stall_; }

private:
    uint32_t seen_  = 0;		// 已处理的帧序号
    uint32_t stall_ = 0;		// 连续无新帧拍数
    uint32_t limit_ = 100;		// 停更判定拍数上限
};

/**
 * @brief 电机掉线检测：按拍数分频调用 PwrLossCheck
 *
 * 分频是为了让检测窗口覆盖若干拍，单拍无反馈不立即判掉线。
 */
class LostWatch
{
public:
    /**
     * @brief 分频周期
     * @param period 每多少拍检测一次
     */
    explicit LostWatch(uint32_t period) : period_(period == 0 ? 1 : period) {}

    /**
     * @brief 本拍是否检测到掉线
     * @param motor 目标电机
     * @return true=掉线
     */
    bool Update(DmMotor& motor)
    {
        if (++count_ < period_) return false;

        count_ = 0;

        return motor.PwrLossCheck();
    }

private:
    uint32_t period_;			// 检测分频拍数
    uint32_t count_ = 0;		// 当前累计拍数
};

/**
 * @brief 发一帧 DM 命令帧（使能/失能等）
 * @param msgq   目标 CAN 发送队列
 * @param can_id 电机控制帧 ID
 * @param data   8 字节命令帧
 */
void SendCmd(k_msgq* msgq, uint16_t can_id, const uint8_t (&data)[8]);

/**
 * @brief 电机是否处于使能态
 * @param motor 目标电机
 * @return true=已使能
 */
bool Enabled(const DmMotor& motor);

/**
 * @brief 使能电机：清错误 → 等 1s → 读状态确认 → 使能 → 等 1s
 * @param motor  目标电机
 * @param msgq   目标 CAN 发送队列
 * @param can_id 电机控制帧 ID
 * @return true=已使能；false=清错未生效或使能失败
 */
bool Enable(DmMotor& motor, k_msgq* msgq, uint16_t can_id);

/**
 * @brief 下发设零并等 1s 停稳
 * @param motor  目标电机
 * @param msgq   目标 CAN 发送队列
 * @param can_id 电机控制帧 ID
 */
void SaveZero(DmMotor& motor, k_msgq* msgq, uint16_t can_id);

} // namespace instance::gimbal
