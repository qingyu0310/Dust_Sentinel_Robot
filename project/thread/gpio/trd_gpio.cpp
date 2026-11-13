/**
 * @file trd_gpio.cpp
 * @author qingyu
 * @brief
 * @version 0.4
 * @date 2026-10-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma message "Compiling Thread/Gpio"

#include "thread.hpp"
#include "Init_entry.hpp"
#include "output.hpp"
#include "timer.hpp"

namespace thread::output {

static Thread<512> thread_{};

static Output heart_beat{};

/**
 * @brief 任务单拍：推进计时器并翻转心跳
 * @param context 未使用
 * @return Continue=正常
 */
static TaskResult Task(void* context)
{
    (void)context;

    static Timer timer(1000);

    timer.Update();

    timer.Clock(([]() {
        heart_beat.Toggle();
    }));

    return TaskResult::Continue;
}

bool thread_init()
{
	// 心跳引脚由 overlay 的 heart_beat 节点决定（compatible = "heart-beat"）
	static const gpio_dt_spec heartbeat_spec = GPIO_DT_SPEC_GET(DT_NODELABEL(heart_beat), gpios);

    return heart_beat.init(heartbeat_spec);
}

bool thread_start()
{
    thread_.Start(Task, K_MSEC(1), ThreadPrio::Low, nullptr, "output", ThreadFaultConfig{ nullptr, nullptr, ThreadFaultPolicy::Isolate });   // 心跳挂了只杀自己
    return true;
}

REGISTER_INIT(thread_init, PreInit, Low, HaltOnFail, nullptr, "output_init");
REGISTER_THREAD(thread_start, PreThread, ContinueOnFail, "output_start");

} // namespace thread::output


