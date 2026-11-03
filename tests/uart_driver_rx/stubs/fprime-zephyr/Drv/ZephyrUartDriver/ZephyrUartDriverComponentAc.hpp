// Test stub of the autocoded ZephyrUartDriverComponentBase: output ports are forwarded to the test (src/main.cpp).
#ifndef TEST_STUB_ZEPHYR_UART_DRIVER_COMPONENT_AC_HPP
#define TEST_STUB_ZEPHYR_UART_DRIVER_COMPONENT_AC_HPP

#include "Fw/Types/BasicTypes.hpp"

namespace Fw {
class Buffer {
  public:
    using SizeType = FwSizeType;
    Buffer() : m_data(nullptr), m_size(0) {}
    Buffer(U8* data, SizeType size) : m_data(data), m_size(size) {}
    U8* getData() const { return m_data; }
    SizeType getSize() const { return m_size; }
    void setSize(SizeType size) { m_size = size; }

  private:
    U8* m_data;
    SizeType m_size;
};
}  // namespace Fw

namespace Drv {
class ByteStreamStatus {
  public:
    enum T { OP_OK = 0, SEND_RETRY = 1, RECV_NO_DATA = 2, OTHER_ERROR = 3 };
    ByteStreamStatus(T value = OP_OK) : e(value) {}
    operator T() const { return e; }
    T e;
};
}  // namespace Drv

namespace Zephyr {
// Hooks implemented by the test
Fw::Buffer test_allocate(U32 size);
void test_deallocate(Fw::Buffer& buffer);
void test_recv(Fw::Buffer& buffer, const Drv::ByteStreamStatus& status);

class ZephyrUartDriverComponentBase {
  public:
    explicit ZephyrUartDriverComponentBase(const char* const) {}
    virtual ~ZephyrUartDriverComponentBase() = default;

  protected:
    Fw::Buffer allocate_out(FwIndexType, U32 size) { return test_allocate(size); }
    void deallocate_out(FwIndexType, Fw::Buffer& buffer) { test_deallocate(buffer); }
    void recv_out(FwIndexType, Fw::Buffer& buffer, const Drv::ByteStreamStatus& status) { test_recv(buffer, status); }
    bool isConnected_ready_OutputPort(FwIndexType) const { return false; }
    void ready_out(FwIndexType) {}
};
}  // namespace Zephyr

#endif
