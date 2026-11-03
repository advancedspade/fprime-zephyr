// ======================================================================
// \title  ZephyrUartDriver.cpp
// \author ethanchee
// \brief  cpp file for ZephyrUartDriver component implementation class
// ======================================================================


#include "fprime-zephyr/Drv/ZephyrUartDriver/ZephyrUartDriver.hpp"
#include "Fw/Types/BasicTypes.hpp"
#include "Fw/Types/Assert.hpp"
#include <Fw/FPrimeBasicTypes.hpp>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(ZephyrUartDriver, LOG_LEVEL_NONE);

namespace Zephyr {

    // ----------------------------------------------------------------------
    // Construction, initialization, and destruction
    // ----------------------------------------------------------------------

    ZephyrUartDriver ::
        ZephyrUartDriver(
            const char *const compName
        ) : ZephyrUartDriverComponentBase(compName), 
        m_rx_throttled(false),
        m_tx_timeout_ms(0U),
        m_tx_tail_len(0U),
        m_tx_tail_sent(0U),
        m_tx_dropped_frames(0U),
        m_tx_truncated_frames(0U)
    {
    }

    ZephyrUartDriver ::
        ~ZephyrUartDriver()
    {

    }

    void ZephyrUartDriver::configure(const struct device *dev, U32 baud_rate) {
        FW_ASSERT(dev != nullptr);
        m_dev = dev;

        if (!device_is_ready(this->m_dev)) {
            return;
        }

        struct uart_config uart_cfg = {
            .baudrate = baud_rate,
            .parity = UART_CFG_PARITY_NONE,
            .stop_bits = UART_CFG_STOP_BITS_1,
            .data_bits = UART_CFG_DATA_BITS_8,
            .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
        };
        uart_configure(this->m_dev, &uart_cfg);

        ring_buf_init(&this->m_ring_buf, RING_BUF_SIZE, this->m_ring_buf_data);
        uart_irq_callback_user_data_set(this->m_dev, serial_cb, this);

        uart_irq_rx_enable(this->m_dev);
	    uart_irq_tx_disable(this->m_dev);

        if (this->isConnected_ready_OutputPort(0)) {
            this->ready_out(0);
        }
    }

    void ZephyrUartDriver::setTxTimeout(U32 timeout_ms) {
        this->m_tx_timeout_ms = timeout_ms;
    }

    void ZephyrUartDriver::serial_cb(const struct device *dev, void *user_data)
    {
        struct ZephyrUartDriver *self = reinterpret_cast<ZephyrUartDriver *>(user_data);

        while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
            // Only RX is serviced (TX interrupts stay disabled). While throttled, or with nothing to read, leave the
            // loop: a pending source this callback does not consume would otherwise keep it spinning. With the NUS
            // UART (uart_bt) this callback runs on a work queue above every F´ thread, so a spin here starved the
            // whole application while the NUS RX ring stayed full.
            if (self->m_rx_throttled || !uart_irq_rx_ready(dev)) {
                break;
            }
            {
                int recv_len, rb_len;
                uint8_t buffer[SERIAL_BUFFER_SIZE];
                size_t len = MIN(ring_buf_space_get(&self->m_ring_buf),
                         sizeof(buffer));
    
                if (len == 0) {
                    /* Throttle because ring buffer is full */
                    uart_irq_rx_disable(dev);
                    self->m_rx_throttled = true;
                    continue;
                }
    
                recv_len = uart_fifo_read(dev, buffer, len);
                if (recv_len < 0) {
                    LOG_ERR("Failed to read UART FIFO");
                    recv_len = 0;
                };
    
                rb_len = ring_buf_put(&self->m_ring_buf, buffer, recv_len);
                if (rb_len < recv_len) {
                    LOG_ERR("Drop %u bytes", recv_len - rb_len);
                }
    
                LOG_INF("IRQ rx: %d bytes -> ringbuf", rb_len);
            }
        }
    
    }

    // ----------------------------------------------------------------------
    // Handler implementations for user-defined typed input ports
    // ----------------------------------------------------------------------

    void ZephyrUartDriver ::
        schedIn_handler(
            const FwIndexType portNum,
            U32 context
        )
    {
        // Hand everything received since the last call to recv in one buffer. Draining a fixed 64 bytes per call
        // capped the uplink at 64 B x rate (1.28 KB/s at 20 Hz), about an NTRIP RTCM stream, so the ring filled.
        const U32 pending = ring_buf_size_get(&this->m_ring_buf);
        if (pending > 0U) {
            Fw::Buffer recv_buffer = this->allocate_out(0, pending);
            if ((recv_buffer.getData() == nullptr) || (recv_buffer.getSize() == 0U)) {
                // No buffer available: keep the bytes (and the throttle), retry on the next call
                return;
            }
            const U32 recv_size = ring_buf_get(&this->m_ring_buf, recv_buffer.getData(), recv_buffer.getSize());
            if (recv_size == 0U) {
                this->deallocate_out(0, recv_buffer);
            } else {
                recv_buffer.setSize(recv_size);
                LOG_INF("schedIn: %u bytes -> recv_out", recv_size);
                recv_out(0, recv_buffer, Drv::ByteStreamStatus::OP_OK);
            }
        }
        if (this->m_rx_throttled && (ring_buf_space_get(&this->m_ring_buf) > 0U)) {
            // Clear the flag before re-enabling RX: uart_irq_rx_enable() may run serial_cb synchronously (uart_bt
            // submits it to a higher-priority work queue that preempts this thread), and it must see RX unthrottled.
            this->m_rx_throttled = false;
            uart_irq_rx_enable(this->m_dev);
        }
    }

    Drv::ByteStreamStatus ZephyrUartDriver ::
        send_handler(
            const FwIndexType portNum,
            Fw::Buffer &sendBuffer
        )
    {
        LOG_INF("TX: %u bytes", (unsigned int)sendBuffer.getSize());
        const U8* const data = sendBuffer.getData();
        const U32 size = static_cast<U32>(sendBuffer.getSize());
        if (this->m_tx_timeout_ms == 0U) {
            for (U32 i = 0; i < size; i++) {
                uart_poll_out(this->m_dev, data[i]);
            }
            return Drv::ByteStreamStatus::OP_OK;
        }
        // Bounded send: uart_poll_out on a full FIFO waits for as long as the link takes to drain (uart_bt sleeps
        // 1 ms per byte), which can hold the caller (ComQueue's thread) for seconds on a slow BLE link.
        const int64_t deadline = k_uptime_get() + static_cast<int64_t>(this->m_tx_timeout_ms);
        // A frame partly queued by an earlier send goes first: the receiver already has its head
        if (this->m_tx_tail_sent < this->m_tx_tail_len) {
            this->m_tx_tail_sent += this->fillUntil(&this->m_tx_tail[this->m_tx_tail_sent],
                                                    this->m_tx_tail_len - this->m_tx_tail_sent, deadline);
            if (this->m_tx_tail_sent < this->m_tx_tail_len) {
                this->m_tx_dropped_frames++;
                return Drv::ByteStreamStatus::SEND_RETRY;
            }
        }
        this->m_tx_tail_len = 0U;
        this->m_tx_tail_sent = 0U;
        const U32 queued = this->fillUntil(data, size, deadline);
        if ((queued == 0U) && (size > 0U)) {
            this->m_tx_dropped_frames++;
            return Drv::ByteStreamStatus::SEND_RETRY;
        }
        const U32 rest = size - queued;
        if (rest > TX_TAIL_SIZE) {
            this->m_tx_truncated_frames++;
        } else if (rest > 0U) {
            (void)memcpy(this->m_tx_tail, &data[queued], rest);
            this->m_tx_tail_len = rest;
        }
        return Drv::ByteStreamStatus::OP_OK;
    }

    U32 ZephyrUartDriver::fillUntil(const U8* data, U32 size, int64_t deadline) {
        U32 sent = 0U;
        while (sent < size) {
            const int written = uart_fifo_fill(this->m_dev, &data[sent], static_cast<int>(size - sent));
            if (written > 0) {
                sent += static_cast<U32>(written);
                continue;
            }
            if ((written < 0) || (k_uptime_get() >= deadline)) {
                break;
            }
            k_sleep(K_MSEC(1));
        }
        return sent;
    }

    void ZephyrUartDriver ::recvReturnIn_handler(const FwIndexType portNum, Fw::Buffer &returnBuffer) {
        LOG_INF("Buffer deallocated, size=%u", (unsigned int)returnBuffer.getSize());
        this->deallocate_out(0, returnBuffer);
    }

} // end namespace Zephyr
