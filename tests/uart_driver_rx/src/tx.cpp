// ZephyrUartDriver TX path on the emulated UART.
//
// Regression for the comQueue assert on the nRF5340 DK (2026-11-02): about 8 s after a BLE connect the NUS link
// drained slower than the downlink, send blocked in uart_poll_out (uart_bt sleeps while its TX ring is full), and
// ComQueue's thread sat in that send while the 20 Hz rate group and telemetry filled its 20-deep message queue. The
// status ComStub posts back to ComQueue when the send returns then found the queue full: FW_ASSERT, reboot.
// With setTxTimeout a send never waits longer than the timeout on a full TX FIFO, and never puts a partial frame on
// the wire (partial frames showed up as CRC errors on the host).

#include <zephyr/device.h>
#include <zephyr/drivers/serial/uart_emul.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include "fprime-zephyr/Drv/ZephyrUartDriver/ZephyrUartDriver.hpp"

namespace {
const struct device* const s_uart = DEVICE_DT_GET(DT_NODELABEL(euart1));
constexpr U32 TX_FIFO = 256;  // app.overlay euart1 tx-fifo-size

Zephyr::ZephyrUartDriver s_tx_driver("uart_tx");
bool s_configured = false;
bool s_drain_in_callback = false;
U8 s_frame[600];
U8 s_out[1024];
U32 s_out_count = 0;

void drain() {
    s_out_count += uart_emul_get_tx_data(s_uart, &s_out[s_out_count], sizeof(s_out) - s_out_count);
}

// uart_emul calls this on every write: draining here models a link that keeps up
void tx_ready(const struct device*, size_t, void*) {
    if (s_drain_in_callback) {
        drain();
    }
}

void before(void*) {
    if (!s_configured) {
        zassert_true(device_is_ready(s_uart));
        s_tx_driver.configure(s_uart, 115200);
        uart_emul_callback_tx_data_ready_set(s_uart, tx_ready, nullptr);
        s_configured = true;
    }
    for (U32 i = 0; i < sizeof(s_frame); i++) {
        s_frame[i] = static_cast<U8>(i * 7U + 1U);
    }
    s_drain_in_callback = false;
    uart_emul_flush_tx_data(s_uart);
    s_out_count = 0;
    s_tx_driver.setTxTimeout(0U);
    s_tx_driver.m_tx_tail_len = 0;
    s_tx_driver.m_tx_tail_sent = 0;
    s_tx_driver.m_tx_dropped_frames = 0;
    s_tx_driver.m_tx_truncated_frames = 0;
}
}  // namespace

ZTEST_SUITE(uart_driver_tx, nullptr, nullptr, before, nullptr, nullptr);

// Default (no timeout): the blocking uart_poll_out path, unchanged
ZTEST(uart_driver_tx, test_default_path_sends_everything) {
    Fw::Buffer buf(s_frame, 200);
    zassert_equal(s_tx_driver.send_handler(0, buf).e, Drv::ByteStreamStatus::OP_OK);
    drain();
    zassert_equal(s_out_count, 200U);
    zassert_mem_equal(s_out, s_frame, 200);
}

// With a timeout and a link that keeps up, every byte goes out in order and nothing is dropped
ZTEST(uart_driver_tx, test_timeout_link_keeping_up_sends_everything) {
    s_tx_driver.setTxTimeout(20U);
    s_drain_in_callback = true;
    Fw::Buffer buf(s_frame, sizeof(s_frame));
    zassert_equal(s_tx_driver.send_handler(0, buf).e, Drv::ByteStreamStatus::OP_OK);
    zassert_equal(s_out_count, sizeof(s_frame), "sent %u of %u", s_out_count, (unsigned)sizeof(s_frame));
    zassert_mem_equal(s_out, s_frame, sizeof(s_frame));
    zassert_equal(s_tx_driver.m_tx_dropped_frames, 0U);
    zassert_equal(s_tx_driver.m_tx_tail_len, 0U);
}

// A stalled link: the send returns after about the timeout instead of blocking; the FIFO holds the head of the frame
// and the rest waits in the tail, so the frame is not cut short
ZTEST(uart_driver_tx, test_stalled_link_bounds_send_and_keeps_rest_of_frame) {
    s_tx_driver.setTxTimeout(20U);
    Fw::Buffer buf(s_frame, sizeof(s_frame));
    const int64_t start = k_uptime_get();
    zassert_equal(s_tx_driver.send_handler(0, buf).e, Drv::ByteStreamStatus::OP_OK);
    const int64_t took = k_uptime_get() - start;
    zassert_true(took >= 20 && took < 100, "send took %lld ms", took);
    drain();
    zassert_equal(s_out_count, TX_FIFO);
    zassert_mem_equal(s_out, s_frame, TX_FIFO);
    zassert_equal(s_tx_driver.m_tx_tail_len, sizeof(s_frame) - TX_FIFO);
    zassert_equal(s_tx_driver.m_tx_dropped_frames, 0U);
}

// The link recovers: the next send first completes the earlier frame, then sends its own, both whole and in order
ZTEST(uart_driver_tx, test_tail_completes_before_next_frame) {
    s_tx_driver.setTxTimeout(20U);
    Fw::Buffer buf(s_frame, sizeof(s_frame));
    (void)s_tx_driver.send_handler(0, buf);
    drain();
    s_drain_in_callback = true;
    Fw::Buffer next(&s_frame[8], 100);
    zassert_equal(s_tx_driver.send_handler(0, next).e, Drv::ByteStreamStatus::OP_OK);
    zassert_equal(s_out_count, sizeof(s_frame) + 100U);
    zassert_mem_equal(s_out, s_frame, sizeof(s_frame));
    zassert_mem_equal(&s_out[sizeof(s_frame)], &s_frame[8], 100);
    zassert_equal(s_tx_driver.m_tx_tail_len, 0U);
    zassert_equal(s_tx_driver.m_tx_dropped_frames, 0U);
}

// Still stalled when the next frame comes: the earlier frame keeps its place and the new one is dropped whole
// (SEND_RETRY), not interleaved
ZTEST(uart_driver_tx, test_new_frame_dropped_whole_while_tail_pending) {
    s_tx_driver.setTxTimeout(20U);
    Fw::Buffer buf(s_frame, sizeof(s_frame));
    (void)s_tx_driver.send_handler(0, buf);
    Fw::Buffer next(&s_frame[8], 100);
    zassert_equal(s_tx_driver.send_handler(0, next).e, Drv::ByteStreamStatus::SEND_RETRY);
    zassert_equal(s_tx_driver.m_tx_dropped_frames, 1U);
    drain();
    zassert_equal(s_out_count, TX_FIFO, "only the head of the first frame may be on the wire");
    zassert_mem_equal(s_out, s_frame, TX_FIFO);
}

// A full FIFO and no tail: nothing of the frame is queued, it is dropped whole
ZTEST(uart_driver_tx, test_full_fifo_drops_whole_frame) {
    s_tx_driver.setTxTimeout(20U);
    Fw::Buffer fill(s_frame, TX_FIFO);
    zassert_equal(s_tx_driver.send_handler(0, fill).e, Drv::ByteStreamStatus::OP_OK);
    zassert_equal(s_tx_driver.m_tx_tail_len, 0U);
    Fw::Buffer next(&s_frame[8], 50);
    zassert_equal(s_tx_driver.send_handler(0, next).e, Drv::ByteStreamStatus::SEND_RETRY);
    zassert_equal(s_tx_driver.m_tx_dropped_frames, 1U);
    drain();
    zassert_equal(s_out_count, TX_FIFO);
}
