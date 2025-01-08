#ifndef UART_H_
#define UART_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"
#include "types/sigSlot.h"
#include <string>

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    class Uart
    {
    public:

        enum class Mode { Rs232, Rs485, Rs485Terminated, Unknown = 255 };
        enum class Parity { None, Odd, Even, Mark, Space, Unknown = 255 };
        enum class StopBits { One, OneAndHalf, Two, Unknown = 255 };
        enum class Events { Open, Close, Error };
        const std::string name;
        Signal<const uint8_t*, uint_t, uint32_t> rxDataEvent;
        Signal<const uint8_t*, uint_t> txCompleteEvent;
        Callback<Uart::Events> uartEvent;

        Uart(const std::string& name) : name(name) {};
        virtual bool_t open() = 0;
        virtual void close() = 0;
        virtual bool_t isOpen() const = 0;
        virtual bool_t config(uint32_t baudrate, uint8_t dataBits, Uart::Parity parity, Uart::StopBits stopBits) = 0;
        virtual bool_t write(const uint8_t* data, uint_t size, uint32_t baudrate=0) = 0;
        virtual void setRxBuffer(uint8_t* buf, uint_t size) = 0;
    };
}

//--------------------------------------------------------------------------------------------------
#endif
