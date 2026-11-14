/**
 * @file trd_pitch_cmd.cpp
 * @author qingyu
 * @brief 上 pitch 达妙电机命令处理：使能、清错、设零
 * @version 0.2
 * @date 2026-11-14
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
    static uint32_t tx_seq = 0;

    topic::to_can_tx::Message msg{};

    msg.tx_id   = can_id;
    msg.version = ++tx_seq;
    memcpy(msg.data, data, 8);

    const int ret = k_msgq_put(&user_can0_msgq, &msg, K_NO_WAIT);
    if (ret != 0) {
        DUST_LOG_ERR("pitch can0 put fail ver=%u id=0x%x", static_cast<unsigned>(msg.version), static_cast<unsigned>(msg.tx_id));
    }
}

/**
 * @brief 上 pitch 是否处于使能态
 * @return true=已使能
 */
bool Enabled()
{
    return up_pitch_motor_.ReadAll().err == DmErrorStatus::Enable;
}

/**
 * @brief 使能上 pitch：清错误 → 等 1s → 读状态确认 → 使能 → 等 1s
 * @return true=已使能；false=清错未生效或使能失败
 */
bool Enable()
{
    uint8_t frame[8];

    const DmMotor::Snapshot up = up_pitch_motor_.ReadAll();

    DUST_LOG_WRN("pitch not enabled: up=%d, clearing err", static_cast<int>(up.err));

    if (up.err != DmErrorStatus::Enable) {
        up_pitch_motor_.PackCmdFrame(frame, DmMotor::Cmd::ClearErr);
        SendCmd(kUpPitchCanId, frame);
        k_msleep(kSettleMs);

        up_pitch_motor_.SetTargetTorque(0.0f);
        up_pitch_motor_.PackCtrlFrame(frame);
        SendCmd(kUpPitchCanId, frame);
        k_msleep(kSettleMs);

        const DmMotor::Snapshot up_after = up_pitch_motor_.ReadAll();

        if (up_after.err != DmErrorStatus::Disable) {
            DUST_LOG_ERR("up pitch clear err failed: err=%d", static_cast<int>(up_after.err));
            return false;
        }

        up_pitch_motor_.PackCmdFrame(frame, DmMotor::Cmd::Enable);
        SendCmd(kUpPitchCanId, frame);
        k_msleep(kSettleMs);
    }

    return Enabled();
}

/**
 * @brief 下发设零并等 1s 停稳
 */
void SaveZero()
{
    uint8_t frame[8];

    up_pitch_motor_.PackCmdFrame(frame, DmMotor::Cmd::SaveZero);
    SendCmd(kUpPitchCanId, frame);
    DUST_LOG_INF("up pitch save zero");
    k_msleep(kSettleMs);
    up_pitch_motor_.ResetEncoder();
}

} // namespace instance::pitch
