/**
 * @file trd_yaw_coop.cpp
 * @author qingyu
 * @brief 大小 yaw 协同：世界角分配 + 小轴软限
 * @version 0.2
 * @date 2026-11-13
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <algorithm>
#include "trd_yaw_coop.hpp"

#pragma message "Compiling Thread/Yaw/Coop"

namespace instance::yaw {

static constexpr float kPi = 3.14159265f;

/**
 * @brief 角度归一化到 [-π, π]
 * @param a 输入角 rad
 * @return 归一化角 rad
 */
float Coop::Wrap(float a)
{
    while (a >  kPi) a -= 2.0f * kPi;
    while (a < -kPi) a += 2.0f * kPi;
    return a;
}

/**
 * @brief 初始化：两轴参考取当前实际关节角
 * @param big_joint  大 yaw 输出轴关节角 rad
 * @param world_yaw  IMU 世界 yaw rad
 */
void Coop::Init(float big_joint, float world_yaw)
{
    world_ref_ = Wrap(world_yaw);
    big_ref_   = Wrap(big_joint);
}

/**
 * @brief 单拍分配，输出两轴目标角与世界目标角
 * @param remote_yaw   遥控归一化 yaw 输入 [-1,1]
 * @param world_yaw    IMU 世界 yaw rad
 * @param big_joint    大 yaw 输出轴关节角 rad
 * @param small_joint  小 yaw 输出轴关节角 rad
 * @param dt           控制周期 s
 * @param big_target   输出：大 yaw 目标关节角 rad
 * @param small_target 输出：小 yaw 目标关节角 rad
 * @param world_target 输出：世界目标角 rad（小轴闭环用）
 */
void Coop::Update(float remote_yaw,  float world_yaw,
                  float big_joint,   float small_joint,
                  float dt,
                  float& big_target, float& small_target, float& world_target)
{
    // 世界目标角速度积分成世界目标角
    world_ref_ = Wrap(world_ref_ + remote_yaw * kYawRateLimitRad * dt);

    // 底盘朝向：IMU 世界角减两轴关节角
    const float base     = Wrap(world_yaw - big_joint - small_joint);
    const float body_ref = Wrap(world_ref_ - base);

    // 目标需求给到小轴；超出软限的部分推给大轴参考
    const float small_required  = Wrap(body_ref - big_ref_);
    const float small_allocated = std::clamp(small_required, -kSmallSoftLimitRad, kSmallSoftLimitRad);
    const float excess          = small_required - small_allocated;

    if (excess != 0.0f) {
        big_ref_ += excess;
    }
    big_ref_ = Wrap(big_ref_);

    big_target   = big_ref_;
    small_target = small_allocated;
    world_target = world_ref_;
}

} // namespace instance::yaw
