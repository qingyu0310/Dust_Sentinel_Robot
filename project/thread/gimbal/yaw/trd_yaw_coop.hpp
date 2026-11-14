/**
 * @file trd_yaw_coop.hpp
 * @author qingyu
 * @brief 大小 yaw 协同：世界角分配 + 小轴软限
 * @version 0.2
 * @date 2026-11-13
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include "angle.hpp"

namespace instance::yaw {

constexpr float kSmallSoftLimitRad = 45.0f * alg::math::kDegToRad;		// 小 yaw 软限 ±45°
constexpr float kSmallHardLimitRad = 50.0f * alg::math::kDegToRad;		// 小 yaw 硬限 ±50°
constexpr float kYawRateLimitRad   = alg::math::kPi;					// 遥控打满的世界角速度 rad/s
constexpr float kYawTorqueMax      = 1.0f;								// yaw 峰值力矩 N*m

/**
 * @brief 世界角分配器
 *
 * 世界目标角速度积分成世界目标角；底盘朝向由 IMU 世界角减两轴关节角推出；
 * 目标需求超小轴软限的部分推给大轴参考，限位内大轴参考保持不动。
 */
class Coop final
{
public:
    /**
     * @brief 初始化：两轴参考取当前实际关节角
     * @param big_joint  大 yaw 输出轴关节角 rad
     * @param world_yaw  IMU 世界 yaw rad
     */
    void Init(float big_joint, float world_yaw);

    /**
     * @brief 单拍分配，输出大轴参考关节角与世界目标角
     * @param remote_yaw   遥控归一化 yaw 输入 [-1,1]
     * @param world_yaw    IMU 世界 yaw rad
     * @param big_joint    大 yaw 输出轴关节角 rad
     * @param small_joint  小 yaw 输出轴关节角 rad
     * @param dt           控制周期 s
     * @param big_target   输出：大 yaw 参考关节角 rad
     * @param small_target 输出：小 yaw 分配目标角 rad（仅用于诊断）
     * @param world_target 输出：世界目标角 rad
     */
    void Update(float remote_yaw, float world_yaw,
                float big_joint, float small_joint,
                float dt,
                float& big_target, float& small_target, float& world_target);

private:
    float world_ref_ = 0.0f;		// 世界目标角 rad
    float big_ref_   = 0.0f;		// 大 yaw 参考关节角 rad
};

} // namespace instance::yaw
