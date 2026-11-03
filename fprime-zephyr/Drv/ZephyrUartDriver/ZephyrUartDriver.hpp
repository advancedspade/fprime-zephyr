// ======================================================================
// \title  ZephyrUartDriver.hpp
// \author ethanchee
// \brief  hpp file for ZephyrUartDriver component implementation class
// ======================================================================

#ifndef ZephyrUartDriver_HPP
#define ZephyrUartDriver_HPP

#include "fprime-zephyr/Drv/ZephyrUartDriver/ZephyrUartDriverComponentAc.hpp"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>

#define RING_BUF_SIZE 1024

namespace Zephyr {

  class ZephyrUartDriver :
    public ZephyrUartDriverComponentBase
  {

    //! Bytes read from the UART FIFO per pass of serial_cb (stack buffer). schedIn hands the whole ring to
    //! recv in one buffer, so this does not limit the uplink rate.
    static constexpr FwSizeType SERIAL_BUFFER_SIZE = 64;

    public:

        // ----------------------------------------------------------------------
        // Construction, initialization, and destruction
        // ----------------------------------------------------------------------

        //! Construct object ZephyrUartDriver
        //!
        ZephyrUartDriver(
            const char *const compName /*!< The component name*/
        );

        //! Destroy object ZephyrUartDriver
        //!
        ~ZephyrUartDriver();

        void configure(const struct device *dev, U32 baud_rate);

        //! Bound how long send may wait for room in the UART TX FIFO, in ms. 0 (the default) keeps the blocking
        //! uart_poll_out path. Use it on links that can stall, such as BLE NUS (uart_bt): a blocked send holds the
        //! caller (ComQueue's thread) while the rate group keeps filling ComQueue's message queue.
        //!
        //! With a timeout, send writes with uart_fifo_fill and never puts a partial frame on the wire:
        //!  - a frame that is partly queued when the timeout expires keeps its rest in a tail buffer, written first
        //!    by the next send (returns OP_OK);
        //!  - a frame of which nothing could be queued, or that arrives while an earlier tail still cannot be
        //!    written, is dropped whole: counted in m_tx_dropped_frames, and send returns SEND_RETRY.
        //! The caller must treat SEND_RETRY as "dropped" (BluetoothMgr does). Wired straight to ComStub, it would be
        //! retried RETRY_LIMIT times and then reported as a failure, which leaves ComQueue waiting for a ready signal.
        void setTxTimeout(U32 timeout_ms);

    public:

        static void serial_cb(const struct device *dev, void *user_data);

        // ----------------------------------------------------------------------
        // Handler implementations for user-defined typed input ports
        // ----------------------------------------------------------------------

        //! Handler implementation for schedIn
        //!
        void schedIn_handler(
            const FwIndexType portNum, /*!< The port number*/
            U32 context /*!< 
        The call order
        */
        );


        //! Handler implementation for send
        //!
        Drv::ByteStreamStatus send_handler(
            const FwIndexType portNum, /*!< The port number*/
            Fw::Buffer &sendBuffer 
        );

        void recvReturnIn_handler(
            const FwIndexType portNum, /*!< The port number*/
            Fw::Buffer &returnBuffer
        );

        const struct device *m_dev;

        U8 m_ring_buf_data[RING_BUF_SIZE];
        struct ring_buf m_ring_buf;
        bool m_rx_throttled; 

        //! Rest of a partly queued frame (setTxTimeout); larger than any downlink frame (FW_COM_BUFFER_MAX_SIZE + framing)
        static constexpr U32 TX_TAIL_SIZE = 1024;
        U32 m_tx_timeout_ms;          //!< 0: blocking uart_poll_out; else the bound on a send (setTxTimeout)
        U8 m_tx_tail[TX_TAIL_SIZE];
        U32 m_tx_tail_len;            //!< Bytes in m_tx_tail
        U32 m_tx_tail_sent;           //!< Bytes of m_tx_tail already queued
        U32 m_tx_dropped_frames;      //!< Frames dropped whole because the TX FIFO stayed full
        U32 m_tx_truncated_frames;    //!< Frames whose rest exceeded TX_TAIL_SIZE (cannot happen with F´ frames)

    private:
        //! uart_fifo_fill until all of data is queued, the FIFO refuses it, or the deadline passes; returns bytes queued
        U32 fillUntil(const U8* data, U32 size, int64_t deadline);
    };

} // end namespace Zephyr

#endif
