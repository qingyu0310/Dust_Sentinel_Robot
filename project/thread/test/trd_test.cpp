/** @brief 上 pitch 往复辨识：PC 配置并触发，MCU 原控制线程执行。 */
#include <algorithm>
#include <cmath>
#include "pitch_sweep.hpp"
#include "pitch_breakaway.hpp"
#include "trd_pitch.hpp"
#include "angle.hpp"
#include "log.hpp"
#include "var.hpp"

namespace pitch_sweep {
using namespace alg::math;
using namespace instance::pitch;
static int32_t on_ = 0, feed_ = 0, cycles_ = 2;
static float low_deg_ = -40.0f, high_deg_ = -10.0f, speed_deg_s_ = 2.0f;
static bool active_, hold_;
static float ref_, low_, high_, speed_;
static uint32_t previous_ms_, feed_ms_, sample_ms_, start_ms_;
static int32_t previous_feed_, cycle_count_, cycle_limit_;
static unsigned phase_; // 0=移到下端，1=正向，2=反向
static int32_t mode_ = 1, points_ = 7, settle_ms_cfg_ = 2000;
static float hold_deg_ = -30.0f; // 模式 2：PC 指定固定姿态，正常验证重力前馈
static float ramp_nm_s_ = 0.05f, movement_deg_ = 0.3f;
static int32_t run_mode_;
static Breakaway breakaway_;
static float previous_command_, previous_feedback_, holding_bias_, override_value_;
static bool override_;

bool OverrideTorque(float& torque)
{
    if (active_ && override_) { torque = override_value_; return true; }
    return false;
}
float HoldingBias() { return (active_ || hold_) && run_mode_ == 1 ? holding_bias_ : 0.0f; }
bool SuppressGravity() { return (active_ || hold_) && run_mode_ != 2; }

static void Finish(const char* reason, float actual)
{
    DUST_LOG_INF("PTEST_HEART feed=%d seen=%d age=%u", static_cast<int>(feed_),
                 static_cast<int>(previous_feed_), static_cast<unsigned>(previous_ms_-feed_ms_));
    active_ = false;
    hold_ = true;
    on_ = 0;
    override_ = false;
    ref_ = actual;
    DUST_LOG_INF("PTEST_END,%s", reason);
}

void Cancel(const char* reason)
{
    if (active_) DUST_LOG_INF("PTEST_END,%s", reason);
    active_ = hold_ = false;
    override_ = false;
    holding_bias_ = 0;
    on_ = 0;
}

bool Target(uint32_t now, float actual, float& target)
{
    if (on_ != 0 && !active_) {
        // 范围与现有位置环统一采用 IMU pitch，不依赖电机编码器零点。
        const float min_deg = kUpPitchMinRad * kRadToDeg + 2.0f;
        const float max_deg = kUpPitchMaxRad * kRadToDeg - 2.0f;
        unsigned invalid = 0;
        if (!std::isfinite(actual) || !std::isfinite(low_deg_) ||
            !std::isfinite(high_deg_) || !std::isfinite(speed_deg_s_)) invalid |= 1U;
        if (low_deg_ < min_deg || high_deg_ > max_deg) invalid |= 2U;
        if (high_deg_ - low_deg_ < 5.0f) invalid |= 4U;
        if (speed_deg_s_ <= 0 || speed_deg_s_ > 10) invalid |= 8U;
        if (cycles_ < 1 || cycles_ > 10) invalid |= 16U;
        if ((mode_ < 0 || mode_ > 2) || points_ < 2 || points_ > 20 ||
            settle_ms_cfg_ < 500 || settle_ms_cfg_ > 10000 ||
            !std::isfinite(ramp_nm_s_) || ramp_nm_s_ < 0.005f || ramp_nm_s_ > 0.2f ||
            !std::isfinite(movement_deg_) || movement_deg_ < 0.1f || movement_deg_ > 1.0f) invalid |= 64U;
        if (mode_ == 2 && (!std::isfinite(hold_deg_) || hold_deg_ < min_deg || hold_deg_ > max_deg)) invalid |= 128U;
        if (actual < kUpPitchMinRad || actual > kUpPitchMaxRad) invalid |= 32U;
        if (invalid != 0) {
            on_ = 0;
            // 单条最多 4 个 double，避免超过日志的 8 个参数槽。
            DUST_LOG_ERR("PTEST_CHECK mask=%u speed=%.2f cycles=%d", invalid,
                         static_cast<double>(speed_deg_s_), static_cast<int>(cycles_));
            DUST_LOG_ERR("PTEST_RANGE req=%.2f/%.2f allowed=%.2f/%.2f",
                         static_cast<double>(low_deg_), static_cast<double>(high_deg_),
                         static_cast<double>(min_deg), static_cast<double>(max_deg));
            DUST_LOG_ERR("PTEST_POSE imu=%.2f limits=%.2f/%.2f",
                         static_cast<double>(actual * kRadToDeg),
                         static_cast<double>(kUpPitchMinRad * kRadToDeg),
                         static_cast<double>(kUpPitchMaxRad * kRadToDeg));
            DUST_LOG_ERR("PTEST_END,invalid_range_or_pose");
            if (hold_) target = ref_;
            return hold_;
        }
        low_ = low_deg_ * kDegToRad;
        high_ = high_deg_ * kDegToRad;
        speed_ = speed_deg_s_ * kDegToRad;
        cycle_limit_ = cycles_;
        run_mode_ = mode_;
        override_ = false;
        previous_command_ = previous_feedback_ = holding_bias_ = 0;
        ref_ = actual;
        phase_ = 0;
        cycle_count_ = 0;
        previous_ms_ = start_ms_ = feed_ms_ = sample_ms_ = now;
        previous_feed_ = feed_;
        active_ = true;
        hold_ = false;
        DUST_LOG_INF("PTEST_BEGIN");
        if (run_mode_ == 1) {
            breakaway_.Start({low_, high_, speed_, ramp_nm_s_, movement_deg_*kDegToRad,
                              kPitchTorqueMax, static_cast<unsigned>(points_),
                              static_cast<unsigned>(cycles_), static_cast<uint32_t>(settle_ms_cfg_)}, now, actual);
        }
    }
    if (!active_) {
        if (hold_) target = ref_;
        return hold_;
    }
    if (feed_ != previous_feed_) { previous_feed_ = feed_; feed_ms_ = now; }
    const float error = std::fabs(actual - ref_);
    if (on_ == 0) Finish("stopped", actual);
    else if (now - feed_ms_ > 3000) Finish("heartbeat_timeout", actual);
    else if (!std::isfinite(actual)) { Cancel("invalid_feedback"); return false; }
    else if (actual <= kUpPitchMinRad || actual >= kUpPitchMaxRad) Finish("angle_limit", actual);
    else if (error > 10.0f * kDegToRad) Finish("tracking_error", actual);
    else if (now-previous_ms_ > 50) Finish("control_gap", actual);
    else if (run_mode_ == 2) {
        if (!std::isfinite(hold_deg_) || hold_deg_*kDegToRad <= kUpPitchMinRad ||
            hold_deg_*kDegToRad >= kUpPitchMaxRad) Finish("invalid_hold_target", actual);
        else {
            const float step = speed_ * (now-previous_ms_) * 0.001f;
            ref_ += std::clamp(hold_deg_*kDegToRad-ref_, -step, step);
            phase_ = 4;
        }
    }
    else if (run_mode_ == 1) {
        const auto out = breakaway_.Update(now, actual, previous_command_, previous_feedback_);
        ref_ = out.target;
        phase_ = breakaway_.Phase();
        override_ = out.override_torque;
        override_value_ = out.torque;
        holding_bias_ = out.bias;
        if (out.event) {
            DUST_LOG_INF("PBREAK,%u,%u,%d,%d,%d,%d,%d,%u",
                         out.point, out.repeat, out.direction,
                         static_cast<int>(out.start_angle*kRadToDeg*1000),
                         static_cast<int>(out.end_angle*kRadToDeg*1000),
                         static_cast<int>(out.command*1000), static_cast<int>(out.feedback*1000),
                         static_cast<unsigned>(now-start_ms_));
        }
        if (out.error != nullptr) Finish(out.error, actual);
        else if (out.done) Finish("done", actual);
    }
    else {
        const uint32_t elapsed = now - previous_ms_;
        if (elapsed > 50) Finish("control_gap", actual);
        else {
            const float end = phase_ == 1 ? high_ : low_;
            const float step = speed_ * elapsed * 0.001f;
            ref_ += std::clamp(end - ref_, -step, step);
            if (std::fabs(end - ref_) < 1e-5f) {
                if (phase_ == 0) phase_ = 1;
                else if (phase_ == 1) phase_ = 2;
                else if (++cycle_count_ >= cycle_limit_) Finish("done", actual);
                else phase_ = 1;
            }
        }
    }
    previous_ms_ = now;
    target = ref_;
    return true;
}

void Sample(uint32_t now, float actual, float omega, float command, float feedback,
            float pid, float gravity, float gravity_scale)
{
    previous_command_ = command;
    previous_feedback_ = feedback;
    static uint32_t heartbeat_ms;
    if (active_ && now-heartbeat_ms >= 1000) {
        heartbeat_ms = now;
        DUST_LOG_INF("PHEART,%u,%d,%u,%u", static_cast<unsigned>(now-start_ms_),
                     static_cast<int>(previous_feed_), static_cast<unsigned>(now-feed_ms_),
                     static_cast<unsigned>(run_mode_));
    }
    if (!active_ || now - sample_ms_ < 10) return;
    sample_ms_ = now;
    if (run_mode_ == 2) {
        DUST_LOG_INF("PGRAV,%u,%d,%d,%d,%d,%d,%d,%d",
                     static_cast<unsigned>(now-start_ms_),
                     static_cast<int>(ref_*kRadToDeg*1000), static_cast<int>(actual*kRadToDeg*1000),
                     static_cast<int>(pid*1000), static_cast<int>(gravity*1000),
                     static_cast<int>(command*1000), static_cast<int>(feedback*1000),
                     static_cast<int>(gravity_scale*1000));
        return;
    }
    // 100Hz 采集；8 个整数参数适配日志参数池，mdeg/mrad/s/mN*m。
    DUST_LOG_INF("PTEST,%u,%u,%d,%d,%d,%d,%d,%u",
                 static_cast<unsigned>(now - start_ms_), phase_,
                 static_cast<int>(ref_ * kRadToDeg * 1000),
                 static_cast<int>(actual * kRadToDeg * 1000), static_cast<int>(omega * 1000),
                 static_cast<int>(command * 1000), static_cast<int>(feedback * 1000),
                 static_cast<unsigned>(cycle_count_));
}
REGISTER_SHELL_VAR("ptest_on", on_);
REGISTER_SHELL_VAR("ptest_feed", feed_);
REGISTER_SHELL_VAR("ptest_low", low_deg_);
REGISTER_SHELL_VAR("ptest_high", high_deg_);
REGISTER_SHELL_VAR("ptest_speed", speed_deg_s_);
REGISTER_SHELL_VAR("ptest_cycles", cycles_);
REGISTER_SHELL_VAR("ptest_mode", mode_);
REGISTER_SHELL_VAR("ptest_hold", hold_deg_);
REGISTER_SHELL_VAR("ptest_points", points_);
REGISTER_SHELL_VAR("ptest_settle", settle_ms_cfg_);
REGISTER_SHELL_VAR("ptest_ramp", ramp_nm_s_);
REGISTER_SHELL_VAR("ptest_move", movement_deg_);
}
