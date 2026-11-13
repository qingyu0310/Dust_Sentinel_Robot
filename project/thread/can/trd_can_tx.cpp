/**
 * @file trd_can_tx.cpp
 * @author qingyu
 * @brief CAN TX 线程——从 to_can_tx 消息队列取帧发送
 * @version 0.2
 * @date 2026-10-31
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma message "Compiling Thread/Can"

#include <cstring>
#include "thread.hpp"
#include "Init_entry.hpp"
#include "to_can_tx.hpp"
#include "can.hpp"
#include "Irq_handlers.h"
#include "log.hpp"

namespace thread::can {

// k_msgq vs zbus：zbus 多个发布者共用一个 channel 会互相覆盖；
// k_msgq 内部拷贝数据，多 put 一 get 天然支持多发布者，且满时丢帧不阻塞。
static Thread<> thread_{};
static Thread<> thread_can0_{};

static Can user_can1{};
static Can user_can0{};

/**
 * @brief 任务单拍：CAN1 队列取一帧发送，队列空立即返回
 * @param context 未使用
 * @return Continue=正常
 */
static TaskResult Task(void* context)
{
	(void)context;

	topic::to_can_tx::Message msg{};

	if (k_msgq_get(&user_can1_msgq, &msg, K_FOREVER) != 0) {
		return TaskResult::Continue;
	}

	can_frame tx {
		.id = msg.tx_id,
		.dlc = 8,
	};

	std::memcpy(tx.data, msg.data, 8);

	if (!user_can1.Send(&tx)) {
		DUST_LOG_ERR("can1 tx fail id=0x%x", static_cast<unsigned>(tx.id));
	}

	return TaskResult::Continue;
}

/**
 * @brief 任务单拍：CAN0 队列取一帧发送，队列空立即返回
 * @param context 未使用
 * @return Continue=正常
 */
static TaskResult TaskCan0(void* context)
{
	(void)context;

	topic::to_can_tx::Message msg{};

	if (k_msgq_get(&user_can0_msgq, &msg, K_FOREVER) != 0) {
		return TaskResult::Continue;
	}

	can_frame tx {
		.id = msg.tx_id,
		.dlc = 8,
	};

	std::memcpy(tx.data, msg.data, 8);

	if (!user_can0.Send(&tx)) {
		DUST_LOG_ERR("can0 tx fail id=0x%x", static_cast<unsigned>(tx.id));
	}

	return TaskResult::Continue;
}

/**
 * @brief 初始化 user_can0 / user_can1 并挂接收分发回调
 * @return true=成功
 */
bool thread_init()
{
	{
		const device* dev = DEVICE_DT_GET(DT_ALIAS(user_can1));

		if (!device_is_ready(dev)) {
			DUST_LOG_ERR("user_can1 not ready");
			return false;
		}

		const can_filter filter { .id = 0, .mask = 0, .flags = 0 };

		user_can1.Init(dev, filter);
		user_can1.SetRxCallback(user_can1_rx_callback);
		DUST_LOG_INF("user_can1 ready");
	}

	{
		const device* dev = DEVICE_DT_GET(DT_ALIAS(user_can0));

		if (!device_is_ready(dev)) {
			DUST_LOG_ERR("user_can0 not ready");
			return false;
		}

		const can_filter filter { .id = 0, .mask = 0, .flags = 0 };

		user_can0.Init(dev, filter);
		user_can0.SetRxCallback(user_can0_rx_callback);
		DUST_LOG_INF("user_can0 ready");
	}

	return true;
}

/**
 * @brief 启动 CAN TX 线程
 * @return true=成功
 */
bool thread_start()
{
	thread_.Start(Task, K_MSEC(1), ThreadPrio::High, nullptr, "can");
	thread_can0_.Start(TaskCan0, K_MSEC(1), ThreadPrio::High, nullptr, "can0");
	return true;
}

REGISTER_INIT(thread_init, PreInit, High, HaltOnFail, nullptr, "can_init");
REGISTER_THREAD(thread_start, PreThread, ContinueOnFail, "can_start");

} // namespace thread::can
