/**
 * @file trd_gimbal.cpp
 * @author qingyu
 * @brief 云台达妙电机公共命令：使能、清错、设零、发帧
 * @version 0.1
 * @date 2026-11-14
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <cstring>
#include "gimbal.hpp"
#include "to_can_tx.hpp"
#include "log.hpp"

namespace instance::gimbal {

static constexpr uint32_t kSettleMs = 1000;		// 命令之间的等待

/**
 * @brief 发一帧 DM 命令帧（使能/失能等）
 * @param msgq   目标 CAN 发送队列
 * @param can_id 电机控制帧 ID
 * @param data   8 字节命令帧
 */
void SendCmd(k_msgq* msgq, uint16_t can_id, const uint8_t (&data)[8])
{
    static uint32_t tx_seq = 0;

    topic::to_can_tx::Message msg{};

    msg.tx_id   = can_id;
    msg.version = ++tx_seq;
    memcpy(msg.data, data, 8);

    const int ret = k_msgq_put(msgq, &msg, K_NO_WAIT);
    if (ret != 0) {
        DUST_LOG_ERR("gimbal put fail ver=%u id=0x%x", static_cast<unsigned>(msg.version), static_cast<unsigned>(msg.tx_id));
    }
}

/**
 * @brief 电机是否处于使能态
 * @param motor 目标电机
 * @return true=已使能
 */
bool Enabled(const DmMotor& motor)
{
    return motor.ReadAll().err == DmErrorStatus::Enable;
}

/**
 * @brief 使能电机：清错误 → 等 1s → 读状态确认 → 使能 → 等 1s
 * @param motor  目标电机
 * @param msgq   目标 CAN 发送队列
 * @param can_id 电机控制帧 ID
 * @return true=已使能；false=清错未生效或使能失败
 */
bool Enable(DmMotor& motor, k_msgq* msgq, uint16_t can_id)
{
    uint8_t frame[8];

    const DmMotor::Snapshot before = motor.ReadAll();

    DUST_LOG_WRN("gimbal not enabled: err=%d, clearing err", static_cast<int>(before.err));

    if (before.err != DmErrorStatus::Enable) 
	{
        motor.PackCmdFrame(frame, DmMotor::Cmd::ClearErr);
        SendCmd(msgq, can_id, frame);
        k_msleep(kSettleMs);

        motor.SetTargetTorque(0.0f);
        motor.PackCtrlFrame(frame);
        SendCmd(msgq, can_id, frame);
        k_msleep(kSettleMs);

        const DmMotor::Snapshot after = motor.ReadAll();

        if (after.err != DmErrorStatus::Disable) {
            DUST_LOG_ERR("gimbal clear err failed: err=%d", static_cast<int>(after.err));
            return false;
        }

        motor.PackCmdFrame(frame, DmMotor::Cmd::Enable);
        SendCmd(msgq, can_id, frame);
        k_msleep(kSettleMs);
    }

    return Enabled(motor);
}

/**
 * @brief 下发设零并等 1s 停稳
 * @param motor  目标电机
 * @param msgq   目标 CAN 发送队列
 * @param can_id 电机控制帧 ID
 */
void SaveZero(DmMotor& motor, k_msgq* msgq, uint16_t can_id)
{
    uint8_t frame[8];

    motor.PackCmdFrame(frame, DmMotor::Cmd::SaveZero);
    SendCmd(msgq, can_id, frame);
    DUST_LOG_INF("gimbal save zero: id=0x%x", static_cast<unsigned>(can_id));
    k_msleep(kSettleMs);
    motor.ResetEncoder();
}

} // namespace instance::gimbal
