/**
 * @file trd_yaw_cmd.cpp
 * @author qingyu
 * @brief yaw 达妙电机命令处理：使能、清错、设零
 * @version 0.1
 * @date 2026-11-13
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <cstring>
#include "trd_yaw.hpp"
#include "trd_yaw_cmd.hpp"
#include "to_can_tx.hpp"
#include "log.hpp"

namespace instance::yaw {

static constexpr uint32_t kSettleMs = 1000;		// 命令之间的等待

/**
 * @brief 发一帧 DM 命令帧（使能/失能等）
 * @param can_id  电机控制帧 ID
 * @param data    8 字节命令帧
 */
void SendCmd(uint16_t can_id, const uint8_t (&data)[8])
{
    topic::to_can_tx::Message msg{};

    msg.tx_id = can_id;
    memcpy(msg.data, data, 8);

    k_msgq_put((can_id == kBigYawCanId) ? &user_can1_msgq : &user_can0_msgq, &msg, K_NO_WAIT);
}

/**
 * @brief 两台 yaw 是否都处于使能态
 * @return true=都已使能
 */
bool BothEnabled()
{
    return big_yaw_motor_.ReadAll().err == DmErrorStatus::Enable && small_yaw_motor_.ReadAll().err == DmErrorStatus::Enable;
}

/**
 * @brief 使能两台 yaw：清错误 → 等 1s → 读状态确认 → 使能 → 等 1s
 * @return true=已使能；false=清错未生效或使能失败
 */
bool EnableBoth()
{
    uint8_t frame[8];

    const DmMotor::Snapshot big   = big_yaw_motor_  .ReadAll();
    const DmMotor::Snapshot small = small_yaw_motor_.ReadAll();

    DUST_LOG_WRN("yaw not enabled: big=%d small=%d, clearing err", static_cast<int>(big.err), static_cast<int>(small.err));

    big_yaw_motor_.PackCmdFrame(frame, DmMotor::Cmd::ClearErr);
    SendCmd(kBigYawCanId, frame);
    small_yaw_motor_.PackCmdFrame(frame, DmMotor::Cmd::ClearErr);
    SendCmd(kSmallYawCanId, frame);
    k_msleep(kSettleMs);

    // 读回状态：清错没生效就不发使能
    const DmMotor::Snapshot big_after   = big_yaw_motor_  .ReadAll();
    const DmMotor::Snapshot small_after = small_yaw_motor_.ReadAll();

    if (big_after.err != DmErrorStatus::Disable || small_after.err != DmErrorStatus::Disable) {
        DUST_LOG_ERR("yaw clear err failed: big=%d small=%d", static_cast<int>(big_after.err), static_cast<int>(small_after.err));
        return false;
    }

    big_yaw_motor_.PackCmdFrame(frame, DmMotor::Cmd::Enable);
    SendCmd(kBigYawCanId, frame);
    small_yaw_motor_.PackCmdFrame(frame, DmMotor::Cmd::Enable);
    SendCmd(kSmallYawCanId, frame);
    k_msleep(kSettleMs);

    return BothEnabled();
}

/**
 * @brief 下发设零并等 1s 停稳
 * @param big  true=大 yaw，false=小 yaw
 */
void SaveZero(bool big)
{
    uint8_t frame[8];

    if (big) {
        big_yaw_motor_.PackCmdFrame(frame, DmMotor::Cmd::SaveZero);
        SendCmd(kBigYawCanId, frame);
        DUST_LOG_INF("big yaw save zero");
        k_msleep(kSettleMs);
        big_yaw_motor_.ResetEncoder();
    } else {
        small_yaw_motor_.PackCmdFrame(frame, DmMotor::Cmd::SaveZero);
        SendCmd(kSmallYawCanId, frame);
        DUST_LOG_INF("small yaw save zero");
        k_msleep(kSettleMs);
        small_yaw_motor_.ResetEncoder();
    }
}

} // namespace instance::yaw
