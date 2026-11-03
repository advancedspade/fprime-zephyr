// ZephyrUartDriver RX path on the emulated UART.
//
// Regression for the RX livelock seen on the nRF5340 DK (2026-10-30): an NTRIP RTCM uplink over BLE filled the
// driver's 1 KB ring, serial_cb throttled RX, and schedIn re-enabled RX before clearing m_rx_throttled. The NUS UART
// runs the callback on a work queue above the rate group, so serial_cb spun forever (RX pending but "throttled") and
// starved the application. uart_emul behaves the same way: its IRQ work keeps calling the callback while RX is enabled
// and data is pending, on a work queue above the test thread (prj.conf). With the old driver the throttle test never
// returns and twister fails it on the timeout.

#include <zephyr/device.h>
#include <zephyr/drivers/serial/uart_emul.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include "fprime-zephyr/Drv/ZephyrUartDriver/ZephyrUartDriver.hpp"

namespace {
const struct device* const s_uart = DEVICE_DT_GET(DT_NODELABEL(euart0));

constexpr U32 POOL_SIZE = 2048;  // Position Cell / CRKT comms buffer bin size
U8 s_pool[POOL_SIZE];
bool s_no_buffer = false;

U8 s_rx[8192];
U32 s_rx_count = 0;
U32 s_recv_calls = 0;
U32 s_largest_recv = 0;
U8 s_next_tx = 0;  // next pattern byte written to the emulated RX line
U8 s_next_rx = 0;  // next pattern byte expected from recv

Zephyr::ZephyrUartDriver s_driver("uart");
bool s_configured = false;

void put_pattern(U32 count) {
    U8 chunk[256];
    while (count > 0U) {
        const U32 n = MIN(count, sizeof(chunk));
        for (U32 i = 0; i < n; i++) {
            chunk[i] = s_next_tx++;
        }
        zassert_equal(uart_emul_put_rx_data(s_uart, chunk, n), n, "emulated RX FIFO full");
        count -= n;
    }
}

// One rate-group tick: schedIn, then let the (higher-priority) UART work queue run
void tick() {
    s_driver.schedIn_handler(0, 0);
    k_sleep(K_MSEC(5));
}

void check_order(U32 from) {
    for (U32 i = from; i < s_rx_count; i++) {
        zassert_equal(s_rx[i], s_next_rx, "byte %u out of order", i);
        s_next_rx++;
    }
}

void before(void*) {
    if (!s_configured) {
        zassert_true(device_is_ready(s_uart));
        s_driver.configure(s_uart, 115200);
        s_configured = true;
    }
    s_no_buffer = false;
    // Drain whatever a previous test left: tick until a tick delivers nothing
    for (int i = 0; i < 50; i++) {
        const U32 before_count = s_rx_count;
        tick();
        if (s_rx_count == before_count) {
            break;
        }
    }
    s_next_rx = s_next_tx;
    s_rx_count = 0;
    s_recv_calls = 0;
    s_largest_recv = 0;
}
}  // namespace

namespace Zephyr {
Fw::Buffer test_allocate(U32 size) {
    if (s_no_buffer) {
        return Fw::Buffer();
    }
    return Fw::Buffer(s_pool, MIN(size, POOL_SIZE));
}

void test_deallocate(Fw::Buffer&) {}

void test_recv(Fw::Buffer& buffer, const Drv::ByteStreamStatus& status) {
    zassert_equal(status.e, Drv::ByteStreamStatus::OP_OK);
    zassert_true(s_rx_count + buffer.getSize() <= sizeof(s_rx));
    memcpy(&s_rx[s_rx_count], buffer.getData(), buffer.getSize());
    s_rx_count += buffer.getSize();
    s_recv_calls++;
    s_largest_recv = MAX(s_largest_recv, buffer.getSize());
}
}  // namespace Zephyr

ZTEST_SUITE(uart_driver_rx, nullptr, nullptr, before, nullptr, nullptr);

// Everything received since the last schedIn reaches recv in one buffer (was capped at 64 bytes per call).
ZTEST(uart_driver_rx, test_burst_drained_in_one_schedin) {
    put_pattern(900);
    k_sleep(K_MSEC(5));
    tick();
    zassert_equal(s_rx_count, 900U);
    zassert_equal(s_recv_calls, 1U);
    check_order(0);
}

// A burst larger than the ring throttles RX; schedIn drains and re-enables it without the callback spinning, and
// every byte arrives in order.
ZTEST(uart_driver_rx, test_throttle_recovers_without_livelock) {
    put_pattern(3000);
    k_sleep(K_MSEC(5));
    zassert_true(s_driver.m_rx_throttled, "a 3000-byte burst must fill the 1 KB ring");
    for (int i = 0; i < 20 && s_rx_count < 3000U; i++) {
        tick();
    }
    zassert_equal(s_rx_count, 3000U, "received %u of 3000", s_rx_count);
    zassert_false(s_driver.m_rx_throttled);
    zassert_true(s_largest_recv <= 1024U && s_largest_recv > 64U);
    check_order(0);
}

// No buffer from the allocator: the bytes stay in the ring and arrive on the next schedIn.
ZTEST(uart_driver_rx, test_no_buffer_keeps_data) {
    put_pattern(100);
    k_sleep(K_MSEC(5));
    s_no_buffer = true;
    tick();
    zassert_equal(s_rx_count, 0U);
    s_no_buffer = false;
    tick();
    zassert_equal(s_rx_count, 100U);
    check_order(0);
}
