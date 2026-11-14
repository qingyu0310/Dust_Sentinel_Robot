/**
 * @file angle.hpp
 * @author qingyu
 * @brief 角度归一化工具 — 内联实现
 * @version 0.1
 * @date 2026-11-13
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#pragma message "Compiling Algorithm/Math/Angle"

#include <cmath>

namespace alg::math {

constexpr float kPi      = 3.14159265358979f;	// π
constexpr float kTwoPi   = 6.28318530717959f;	// 2π
constexpr float kDegToRad = 0.01745329252f;		// 度转弧度
constexpr float kRadToDeg = 57.2957795f;		// 弧度转度

/**
 * @brief 角度归一化到 [-π, π]
 * @param rad 输入角 rad
 * @return 归一化角 rad
 */
inline float WrapRad(float rad)
{
    rad = std::fmod(rad + kPi, kTwoPi);
    if (rad < 0.0f) {
        rad += kTwoPi;
    }
    return rad - kPi;
}

/**
 * @brief 角度归一化到 [-180, 180]
 * @param deg 输入角（度）
 * @return 归一化角（度）
 */
inline float WrapDeg(float deg)
{
    deg = std::fmod(deg + 180.0f, 360.0f);
    if (deg < 0.0f) {
        deg += 360.0f;
    }
    return deg - 180.0f;
}

} // namespace alg::math
