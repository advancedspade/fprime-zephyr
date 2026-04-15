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

LOG_MODULE_REGISTER(ZephyrUartDriver, LOG_LEVEL_NONE);

namespace Zephyr {

    // ----------------------------------------------------------------------
    // Construction, initialization, and destruction
    // ----------------------------------------------------------------------

    ZephyrUartDriver ::
        ZephyrUartDriver(
            const char *const compName
        ) : ZephyrUartDriverComponentBase(compName), 
        m_rx_throttled(false)
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

    void ZephyrUartDriver::serial_cb(const struct device *dev, void *user_data)
    {
        struct ZephyrUartDriver *self = reinterpret_cast<ZephyrUartDriver *>(user_data);

        while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
            if (!self->m_rx_throttled && uart_irq_rx_ready(dev)) {
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
        if (ring_buf_is_empty(&this->m_ring_buf)) {
            return; 
        }
        Fw::Buffer recv_buffer = this->allocate_out(0, SERIAL_BUFFER_SIZE);

        U32 recv_size = ring_buf_get(&this->m_ring_buf, recv_buffer.getData(), recv_buffer.getSize());
        if (recv_size == 0) {
            // No data received, deallocate buffer
            this->deallocate_out(0, recv_buffer);
        } else {
            recv_buffer.setSize(recv_size);
            LOG_INF("schedIn: %u bytes -> recv_out", recv_size);
            recv_out(0, recv_buffer, Drv::ByteStreamStatus::OP_OK);
        }
        if (this->m_rx_throttled) {
            uart_irq_rx_enable(this->m_dev);
            this->m_rx_throttled = false;
        }
    }

    Drv::ByteStreamStatus ZephyrUartDriver ::
        send_handler(
            const FwIndexType portNum,
            Fw::Buffer &sendBuffer
        )
    {
        LOG_INF("TX: %u bytes", (unsigned int)sendBuffer.getSize());
        for (U32 i = 0; i < sendBuffer.getSize(); i++) {
            uart_poll_out(this->m_dev, sendBuffer.getData()[i]);
        }
        return Drv::ByteStreamStatus::OP_OK;
    }

    void ZephyrUartDriver ::recvReturnIn_handler(const FwIndexType portNum, Fw::Buffer &returnBuffer) {
        LOG_INF("Buffer deallocated, size=%u", (unsigned int)returnBuffer.getSize());
        this->deallocate_out(0, returnBuffer);
    }

} // end namespace Zephyr
