/**
 * @file trd_test.cpp
 * @author qingyu
 * @brief 测试线程 —— 订阅 IMU 话题打印欧拉角
 * @version 0.3
 * @date 2026-11-13
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma message "Compiling Thread/Test"

#include <zephyr/zbus/zbus.h>
#include "thread.hpp"
#include "Init_entry.hpp"
#include "imu_to.hpp"
#include "log.hpp"

namespace thread::test {

static Thread<2048> thread_{};

/**
 * @brief 任务单拍：订阅 IMU 话题，打印欧拉角
 * @param context 未使用
 * @return Continue=正常
 */
static TaskResult Task(void* context)
{
	(void)context;

	static topic::imu_to::Message msg {};
	const zbus_channel* chan = nullptr;

	zbus_sub_wait(&sub_imu_to, &chan, K_MSEC(1));

	if (chan != nullptr) {
		zbus_chan_read(chan, &msg, K_NO_WAIT);
	}

	DUST_LOG_DBG("euler", "%.2f,%.2f,%.2f", static_cast<double>(msg.roll), static_cast<double>(msg.pitch), static_cast<double>(msg.yaw));

	return TaskResult::Continue;
}

/**
 * @brief 测试线程无初始化
 * @return true=成功
 */
bool thread_init()
{
	return true;
}

/**
 * @brief 启动测试线程
 * @return true=成功
 */
bool thread_start()
{
	thread_.Start(Task, K_MSEC(5), ThreadPrio::High, nullptr, "test", ThreadFaultConfig{ nullptr, nullptr, ThreadFaultPolicy::Isolate });
	return true;
}

REGISTER_INIT(thread_init, LateInit, High, HaltOnFail, nullptr, "test_init");
REGISTER_THREAD(thread_start, LateThread, ContinueOnFail, "test_start");

} // namespace thread::test
