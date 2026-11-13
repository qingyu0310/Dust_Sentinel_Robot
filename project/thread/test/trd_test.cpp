/**
 * @file trd_test.cpp
 * @author qingyu
 * @brief 测试线程 —— C620 电流阶跃辨识（逐台电机跑正反阶跃，采点给 PC 拟合 K/tau）
 * @version 0.2
 * @date 2026-11-13
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma message "Compiling Thread/Test"

#include "trd_test.hpp"
#include "thread.hpp"
#include "Init_entry.hpp"
#include "to_can_tx.hpp"
#include "dji_c6xx.hpp"
#include "log.hpp"
#include "var.hpp"

namespace thread::test {

using namespace instance::test;

static Thread<2048> thread_{};

static motor::dji::DjiC620 motor_[N_Motor] {};

static float    g_step_i    = 1.0f;		// 阶跃电流（A，var 可调）
static uint32_t g_hold_ms   = 6000;		// 阶跃持续时间（ms，var 可调）
static int32_t  g_test_motor = 0;		// 只测这一台（0~3，var 可调）
static int32_t  g_test_on   = 0;		// 1 = 跑一轮辨识，跑完自清零（var 可调）

/**
 * @brief 发一帧控制帧：指定电机给 u，其余电机给 0
 * @param active  通电流的电机索引
 * @param u       目标电流 A
 */
static void SendCurrent(uint8_t active, float u)
{
	topic::to_can_tx::Message tx{};

	tx.tx_id = kControlTxId;

	for (uint8_t i = 0; i < N_Motor; ++i) {
		motor_[i].PackCurrent(tx.data, (i == active) ? u : 0.0f);
	}

	k_msgq_put(&user_can1_msgq, &tx, K_NO_WAIT);
}

/**
 * @brief 任务单拍：test_on 置 1 时逐台电机跑正反阶跃并逐毫秒打点
 * @param context 未使用
 * @return Continue=正常
 */
static TaskResult Task(void* context)
{
	(void)context;

	if (g_test_on == 0) {
		return TaskResult::Continue;
	}
	g_test_on = 0;

	// 多档电流扫描：判惯性主导 vs 摩擦主导。稳态转速随电流亚线性 = 摩擦主导
	static constexpr float kLevels[] = { 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, -1.0f, -2.0f, -3.0f, -4.0f, -5.0f };

	const uint8_t m = (g_test_motor < 0) ? 0 : (g_test_motor >= N_Motor ? N_Motor - 1 : static_cast<uint8_t>(g_test_motor));

	for (uint8_t s = 0; s < sizeof(kLevels) / sizeof(kLevels[0]); ++s)
	{
		const float u = kLevels[s];

		// 先在 0 电流下采 200ms 基线，再跳到 u 保持 g_hold_ms
		const uint32_t t0 = k_uptime_get_32();

		for (uint32_t t = 0; t < 200 + g_hold_ms; ) {
			const float cmd = (t < 200) ? 0.0f : u;

			SendCurrent(m, cmd);

			const float omega = motor_[m].ReadAll().omega;

			DUST_LOG_INF("m%u,%u,%d,%d", static_cast<unsigned>(m), static_cast<unsigned>(k_uptime_get_32()), static_cast<int>(cmd * 1000.0f), static_cast<int>(omega * 1000.0f));

			k_msleep(1);
			t = k_uptime_get_32() - t0;
		}

		// 两次阶跃之间：归零并轮询等电机真正停稳（|omega| < 2 rad/s），上限 10s
		const uint32_t t_stop = k_uptime_get_32();

		while (k_uptime_get_32() - t_stop < 10000)
		{
			SendCurrent(m, 0.0f);

			const float omega = motor_[m].ReadAll().omega;

			DUST_LOG_INF("m%u,%u,0,%d", static_cast<unsigned>(m), static_cast<unsigned>(k_uptime_get_32()), static_cast<int>(omega * 1000.0f));

			if (k_uptime_get_32() - t_stop > 300 && omega > -2.0f && omega < 2.0f) {
				break;
			}

			k_msleep(1);
		}
	}

	DUST_LOG_INF("m%u done", static_cast<unsigned>(m));

	// 收尾：电流归零
	for (uint8_t i = 0; i < N_Motor; ++i) {
		SendCurrent(i, 0.0f);
	}

	return TaskResult::Continue;
}

/**
 * @brief 初始化四个辨识电机
 * @return true=成功
 */
bool thread_init()
{
	motor::dji::DjiC620::Config motor_cfg{};

	motor_cfg.gearbox_ratio = 3542.0f / 238.0f;
	motor_cfg.wheel_r       = 0.570f * 0.5f;

	for (uint8_t i = 0; i < N_Motor; ++i) {
		motor_cfg.rx_id = kTestCanId[i];

		motor_[i].Init(motor_cfg);
	}

	DUST_LOG_INF("test ready");

	return true;
}

/**
 * @brief 启动辨识线程
 * @return true=成功
 */
bool thread_start()
{
	thread_.Start(Task, K_MSEC(1), ThreadPrio::High, nullptr, "test", ThreadFaultConfig{ nullptr, nullptr, ThreadFaultPolicy::Isolate });
	return true;
}

REGISTER_INIT(thread_init, LateInit, High, HaltOnFail, nullptr, "test_init");
REGISTER_THREAD(thread_start, LateThread, ContinueOnFail, "test_start");

REGISTER_SHELL_VAR("step_i", g_step_i);
REGISTER_SHELL_VAR("hold_ms", g_hold_ms);
REGISTER_SHELL_VAR("test_motor", g_test_motor);
REGISTER_SHELL_VAR("test_on", g_test_on);

CAN_RX_HANDLER(TEST_RX, kTestCanId[0], [](uint8_t* data) { motor_[0].CanCpltRxCallback(data); });
CAN_RX_HANDLER(TEST_RX, kTestCanId[1], [](uint8_t* data) { motor_[1].CanCpltRxCallback(data); });
CAN_RX_HANDLER(TEST_RX, kTestCanId[2], [](uint8_t* data) { motor_[2].CanCpltRxCallback(data); });
CAN_RX_HANDLER(TEST_RX, kTestCanId[3], [](uint8_t* data) { motor_[3].CanCpltRxCallback(data); });

} // namespace thread::test
