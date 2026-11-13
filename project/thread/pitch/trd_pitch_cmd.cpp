/**
 * @file trd_pitch_cmd.cpp
 * @author qingyu
 * @brief pitch 达妙电机命令处理：使能、清错、设零
 * @version 0.1
 * @date 2026-11-13
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <cstring>
#include "trd_pitch.hpp"
#include "trd_pitch_cmd.hpp"
#include "to_can_tx.hpp"
#include "log.hpp"

namespace instance::pitch {

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

    k_msgq_put(&user_can0_msgq, &msg, K_NO_WAIT);
}

/**
 * @brief 两台 pitch 是否都处于使能态
 * @return true=都已使能
 */
bool BothEnabled()
{
    return up_pitch_motor_.ReadAll().err == DmErrorStatus::Enable
        && down_pitch_motor_.ReadAll().err == DmErrorStatus::Enable;
}

/**
 * @brief 使能两台 pitch：清错误 → 等 1s → 读状态确认 → 使能 → 等 1s
 * @return true=已使能；false=清错未生效或使能失败
 */
bool EnableBoth()
{
    uint8_t frame[8];

    const DmMotor::Snapshot up   = up_pitch_motor_  .ReadAll();
    const DmMotor::Snapshot down = down_pitch_motor_.ReadAll();

    DUST_LOG_WRN("pitch not enabled: up=%d down=%d, clearing err", static_cast<int>(up.err), static_cast<int>(down.err));

    up_pitch_motor_.PackCmdFrame(frame, DmMotor::Cmd::ClearErr);
    SendCmd(kUpPitchCanId, frame);
    down_pitch_motor_.PackCmdFrame(frame, DmMotor::Cmd::ClearErr);
    SendCmd(kDownPitchCanId, frame);
    k_msleep(kSettleMs);

    // 读回状态：清错没生效就不发使能
    const DmMotor::Snapshot up_after   = up_pitch_motor_  .ReadAll();
    const DmMotor::Snapshot down_after = down_pitch_motor_.ReadAll();

    if (up_after.err != DmErrorStatus::Disable || down_after.err != DmErrorStatus::Disable) {
        DUST_LOG_ERR("pitch clear err failed: up=%d down=%d", static_cast<int>(up_after.err), static_cast<int>(down_after.err));
        return false;
    }

    up_pitch_motor_.PackCmdFrame(frame, DmMotor::Cmd::Enable);
    SendCmd(kUpPitchCanId, frame);
    down_pitch_motor_.PackCmdFrame(frame, DmMotor::Cmd::Enable);
    SendCmd(kDownPitchCanId, frame);
    k_msleep(kSettleMs);

    return BothEnabled();
}

/**
 * @brief 下发设零并等 1s 停稳
 * @param up  true=上 pitch，false=下 pitch
 */
void SaveZero(bool up)
{
    uint8_t frame[8];

    if (up) {
        up_pitch_motor_.PackCmdFrame(frame, DmMotor::Cmd::SaveZero);
        SendCmd(kUpPitchCanId, frame);
        DUST_LOG_INF("up pitch save zero");
        k_msleep(kSettleMs);
        up_pitch_motor_.ResetEncoder();
    } else {
        down_pitch_motor_.PackCmdFrame(frame, DmMotor::Cmd::SaveZero);
        SendCmd(kDownPitchCanId, frame);
        DUST_LOG_INF("down pitch save zero");
        k_msleep(kSettleMs);
        down_pitch_motor_.ResetEncoder();
    }
}

} // namespace instance::pitch
