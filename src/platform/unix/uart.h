#ifndef UNIX_UART_H_
#define UNIX_UART_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"
#include "types/sigSlot.h"
#include <string>
#include <vector>
#include <thread>

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    class Uart
    {
    public:
        enum class Parity { None, Odd, Even, Mark, Space, Unknown };
        enum class StopBits { One, OneAndHalf, Two, Unknown };
        const std::string name;
        Signal<const uint8_t*, uint_t, uint32_t> rxDataEvent;
        Signal<const uint8_t*, uint_t> txCompleteEvent;
        Callback<> errorEvent;

        static std::vector<std::string> getNames();
        Uart(const std::string& name);
        ~Uart();
        bool_t open();
        void close();
        bool_t config(uint32_t baudrate, uint8_t dataBits, Uart::Parity parity, Uart::StopBits stopBits);
        bool_t write(const uint8_t* data, uint_t size, uint32_t baudrate);
        void setRxBuffer(uint8_t* buf, uint_t size);

    private:
        int_t m_portHandle;
        uint8_t* m_rxBuf;
        uint_t m_rxBufSize;
        uint32_t m_txBaudrate;
        std::thread m_threadRx;
        uint32_t m_baudrate;
        uint8_t m_dataBits;
        Uart::Parity m_parity;
        Uart::StopBits m_stopBits;
        bool_t m_runRxthread;
        void threadRx();
    };
}
//--------------------------------------------------------------------------------------------------
#endif
