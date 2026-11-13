/**
 * @file trd_chassis.cpp
 * @author qingyu
 * @brief 全向轮底盘线程 —— 遥控速度 → 逆运动学 → C620 角速度环
 * @version 0.1
 * @date 2026-11-01
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma message "Compiling Thread/Chassis"

#include <cmath>
#include <cstdint>
#include <zephyr/zbus/zbus.h>
#include "trd_chassis.hpp"
#include "thread.hpp"
#include "Init_entry.hpp"
#include "remote_to.hpp"
#include "to_can_tx.hpp"
#include "pid.hpp"
#include "log.hpp"
#include "var.hpp"

namespace thread::chassis {

using namespace instance::chassis;

static Thread<2048> thread_{};
static alg::pid::Pid pid_[N_Wheel] {};

/**
 * @brief 任务单拍：读遥控速度 → 逆运动学 → 四轮角速度环 → 组帧进 CAN 发送队列
 * @param context 未使用
 * @return Continue=正常
 */
static TaskResult Task(void* context)
{
	(void)context;

	static topic::remote_to::Message msg{};
	const zbus_channel* chan = nullptr;

	zbus_sub_wait(&sub_remote_to, &chan, K_MSEC(1));

	if (chan != nullptr) {
		zbus_chan_read(chan, &msg, K_NO_WAIT);
	}

	DUST_LOG_DBG("wheel12", "%.2f,%.2f,%.2f,%.2f", static_cast<double>(chassis_motor_[0].ReadAll().omega), static_cast<double>(chassis_motor_[1].ReadAll().omega),
															 static_cast<double>(chassis_motor_[2].ReadAll().omega), static_cast<double>(chassis_motor_[3].ReadAll().omega));

	// 逆运动学反解：轮角速度 → 底盘实际速度 (m/s)
	{
		static constexpr float kInv4Sqrt2 = 0.25f * 0.70710678f;			// 1/(4√2)

		const float w0 = chassis_motor_[0].ReadAll().omega;
		const float w1 = chassis_motor_[1].ReadAll().omega;
		const float w2 = chassis_motor_[2].ReadAll().omega;
		const float w3 = chassis_motor_[3].ReadAll().omega;

		const float vx = (-w0 - w1 + w2 + w3) * kInv4Sqrt2 * kWheelRadius;
		const float vy = ( w0 - w1 - w2 + w3) * kInv4Sqrt2 * kWheelRadius;

		DUST_LOG_DBG("chassis_v", "%.3f,%.3f", static_cast<double>(vx), static_cast<double>(vy));
	}

	float current[N_Wheel] {};

	if (msg.chassis_ctrl == topic::remote_to::StartMode::Off)
	{
		for (uint8_t i = 0; i < N_Wheel; ++i)
		{
			current[i] = 0.0f;
		}
	}
	else
	{
		static constexpr float kSqrt2 = 1.41421356f;                    		// 对角线 → 边投影

		// 轮角速度 = 底盘线速度 / 轮半径
		const float vx =  msg.chassisx * chassis_v / kWheelRadius;
		const float vy =  msg.chassisy * chassis_v / kWheelRadius;
		const float wz = (msg.chassis_mode == topic::remote_to::ChassisMode::Spin) ? chassis_w : 0.0f;

		float target[N_Wheel] {};

		target[0] = (-vx + vy) * kSqrt2 + wz;
		target[1] = (-vx - vy) * kSqrt2 + wz;
		target[2] = ( vx - vy) * kSqrt2 + wz;
		target[3] = ( vx + vy) * kSqrt2 + wz;

		for (uint8_t i = 0; i < N_Wheel; ++i)
		{
			const float omega = chassis_motor_[i].ReadAll().omega;

			current[i] = pid_[i].Calc(target[i], omega);
		}
	}

	topic::to_can_tx::Message tx{};

	tx.tx_id = kControlTxId;

	for (uint8_t i = 0; i < N_Wheel; ++i)
	{
		chassis_motor_[i].PackCurrent(tx.data, current[i]);
	}

	k_msgq_put(&user_can1_msgq, &tx, K_NO_WAIT);

	return TaskResult::Continue;
}

/**
 * @brief 初始化四个 C620 与角速度环
 * @return true=成功
 */
bool thread_init()
{
	static constexpr float kGearRatio = 3542.0f / 238.0f;

	motor::dji::DjiC620::Config motor_cfg{};

	// 每轮一套 IMC 参数：由 scripts/motor_ident.py 实测，四轮减速箱精度不同不能共用
	// 纯 P 试：ki = 0，看换向响应和稳态误差
	static constexpr float kKp[N_Wheel] = { 0.2745f, 0.2598f, 0.8890f, 0.5103f };
	alg::pid::Pid::Config pid_cfg{};

	pid_cfg.kd      = 0.0f;
	pid_cfg.outMax  = 5.0f;   // C620 电流上限
	pid_cfg.iOutMax = 2.0f;   // 积分项限幅，防止换向时积分饱和拖慢响应

	for (uint8_t i = 0; i < N_Wheel; ++i)
	{
		motor_cfg.rx_id         = kDriveCanId[i];
		motor_cfg.gearbox_ratio = kGearRatio;
		motor_cfg.wheel_r       = kWheelRadius;

		pid_cfg.kp = kKp[i];

		chassis_motor_[i].Init(motor_cfg);
		pid_[i].Init(pid_cfg);
	}

	DUST_LOG_INF("chassis ready");

	return true;
}

/**
 * @brief 启动底盘线程
 * @return true=成功
 */
bool thread_start()
{
	thread_.Start(Task, K_MSEC(1), ThreadPrio::High, nullptr, "chassis");
	return true;
}

REGISTER_INIT(thread_init, PreInit, High, HaltOnFail, nullptr, "chassis_init");
REGISTER_THREAD(thread_start, PreThread, ContinueOnFail, "chassis_start");

REGISTER_SHELL_VAR("chassis_v", chassis_v);
REGISTER_SHELL_VAR("chassis_w", chassis_w);

// C620 反馈注册（CAN1 分发）
CAN_RX_HANDLER(CHASSIS_RX, kDriveCanId[0], [](uint8_t* data) { chassis_motor_[0].CanCpltRxCallback(data); });
CAN_RX_HANDLER(CHASSIS_RX, kDriveCanId[1], [](uint8_t* data) { chassis_motor_[1].CanCpltRxCallback(data); });
CAN_RX_HANDLER(CHASSIS_RX, kDriveCanId[2], [](uint8_t* data) { chassis_motor_[2].CanCpltRxCallback(data); });
CAN_RX_HANDLER(CHASSIS_RX, kDriveCanId[3], [](uint8_t* data) { chassis_motor_[3].CanCpltRxCallback(data); });

} // namespace thread::chassis
