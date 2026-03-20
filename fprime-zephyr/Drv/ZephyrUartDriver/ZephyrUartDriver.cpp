// ======================================================================
// \title  ZephyrUartDriver.cpp
// \author ethanchee
// \brief  cpp file for ZephyrUartDriver component implementation class
// ======================================================================


#include "fprime-zephyr/Drv/ZephyrUartDriver/ZephyrUartDriver.hpp"
#include "Fw/Types/BasicTypes.hpp"
#include "Fw/Types/Assert.hpp"
#include <Fw/FPrimeBasicTypes.hpp>
#include "aspade/core/Dispatcher.hpp"

namespace Zephyr {

    // ----------------------------------------------------------------------
    // Construction, initialization, and destruction
    // ----------------------------------------------------------------------

    ZephyrUartDriver ::
        ZephyrUartDriver(
            const char *const compName
        ) : ZephyrUartDriverComponentBase(compName)
    {
    }

    ZephyrUartDriver ::
        ~ZephyrUartDriver()
    {

    }

    void ZephyrUartDriver::configure(const struct device *dev, U32 baud_rate) {
        if (this->isConnected_ready_OutputPort(0)) {
            this->ready_out(0);
        }
    }

    void ZephyrUartDriver::serial_cb(const struct device *dev, void *user_data)
    {
    }

    // ----------------------------------------------------------------------
    // Handler implementations for user-defined typed input ports
    // ----------------------------------------------------------------------

    /**
     * @brief Process F-Prime formatted inbound messages that have come in over the
     * client connection.
     */
    void ZephyrUartDriver ::
        schedIn_handler(
            const FwIndexType portNum,
            U32 context
        )
    {
        AdvancedSpade::Dispatcher& d = AdvancedSpade::Dispatcher::getInstance();
        uint8_t bytes[FPRIME_BUFFER_CAPACITY];
        size_t size;
        d.getFprimeBuffer(bytes, size);
        if (size > 0) {
            Fw::Buffer recv_buffer = this->allocate_out(0, SERIAL_BUFFER_SIZE);
            recv_buffer.setSize(size);
            memcpy(recv_buffer.getData(), bytes, size);
            recv_out(0, recv_buffer, Drv::ByteStreamStatus::OP_OK);
        }
    }

    /**
     * @brief Send F-Prime outbound messages over the client connection.
     */
    Drv::ByteStreamStatus ZephyrUartDriver ::
        send_handler(
            const FwIndexType portNum,
            Fw::Buffer &sendBuffer
        )
    {
        AdvancedSpade::Dispatcher& d = AdvancedSpade::Dispatcher::getInstance();
        size_t count = sendBuffer.getSize();
        const uint8_t* data = sendBuffer.getData();
        d.send(data, count);
        return Drv::ByteStreamStatus::OP_OK;
    }

    void ZephyrUartDriver ::recvReturnIn_handler(const FwIndexType portNum, Fw::Buffer &returnBuffer) {
        this->deallocate_out(0, returnBuffer);
    }

} // end namespace Zephyr
