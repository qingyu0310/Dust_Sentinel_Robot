/**
 * @file remote_map_dr16.cpp
 * @author qingyu
 * @brief DR16 遥控映射 — 覆盖框架默认后处理
 * @version 0.1
 * @date 2026-11-01
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "dr16.hpp"

namespace dr16 {

/**
 * @brief DR16 后处理：sw2 上为小陀螺，其余正常模式
 * @param in   协议输出
 * @param out  已被默认映射填好的业务消息
 */
void FillOutputData(const OutputData& in, topic::remote_to::Message& out)
{
    out.chassis_mode = (in.sw.sw2 == SWITCH_UP) ? topic::remote_to::ChassisMode::Spin : topic::remote_to::ChassisMode::Normal;
    out.robot_ctrl   = (in.sw.sw1 == SWITCH_UP) ? topic::remote_to::StartMode::Off 	  : topic::remote_to::StartMode::On;
}

} // namespace dr16
