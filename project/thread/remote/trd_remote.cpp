/**
 * @file trd_remote.cpp
 * @author qingyu
 * @brief 遥控器线程 — UART 接收，交给 remote 模块解析
 * @version 0.2
 * @date 2026-10-19
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma message "Compiling Thread/Remote"

#include "remote.hpp"
#include "Init_entry.hpp"
#include "uart.hpp"
#include "log.hpp"
#include <zephyr/kernel.h>

namespace thread::remote {

static ::remote::Remote remote_ {};

bool thread_init()
{
    static UartDma rx {};

    constexpr uint16_t kTimeoutMs = 1000;

    // 初始参数（DR16），协议切换时由 remote 模块 Reconfigure 成对应协议的线路参数
    UartDma::Config cfg 
	{
        .line_cfg = {
            .baudrate  = 100000,
            .parity    = UART_CFG_PARITY_EVEN,
            .stop_bits = UART_CFG_STOP_BITS_2,
            .data_bits = UART_CFG_DATA_BITS_8,
            .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
        },
        .base_cfg = { .rx_timeout = kTimeoutMs },
    };

    if (!rx.Init(DEVICE_DT_GET(DT_ALIAS(remote_uart)), cfg)) {
        DUST_LOG_ERR("uart init failed");
        return false;
    }

    remote_.Init(rx);
	
    return true;
}

bool thread_start()
{
    return remote_.Start(ThreadPrio::High);
}

REGISTER_INIT(thread_init, EarlyInit, High, HaltOnFail, nullptr, "remote_init");
REGISTER_THREAD(thread_start, EarlyThread, ContinueOnFail, "remote_start");

} // namespace thread::remote
