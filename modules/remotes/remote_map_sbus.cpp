/**
 * @file remote_map_sbus.cpp
 * @author qingyu
 * @brief SBUS 遥控映射 — 覆盖框架默认后处理
 * @version 0.1
 * @date 2026-11-13
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "sbus.hpp"

namespace sbus {

/**
 * @brief SBUS 后处理：sa 上关整车/下开整车，sc 上为小陀螺
 * @param in   协议输出
 * @param out  已被默认映射填好的业务消息
 */
void FillOutputData(const OutputData& in, topic::remote_to::Message& out)
{
    out.robot_ctrl   = (in.sw.sa == TwoPos::Up)   ? topic::remote_to::StartMode::Off    : topic::remote_to::StartMode::On;
    out.chassis_mode = (in.sw.sc == ThreePos::Up) ? topic::remote_to::ChassisMode::Spin : topic::remote_to::ChassisMode::Normal;
}

} // namespace sbus
