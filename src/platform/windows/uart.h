#ifndef WINDOWS_UART_H_
#define WINDOWS_UART_H_

//------------------------------------------ Includes ----------------------------------------------

#define WIN32_LEAN_AND_MEAN

#include "types/sdkTypes.h"
#include "types/sigSlot.h"
#include <string>
#include <vector>
#include <thread>
#include "windows.h"

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
        const bool_t& isSending = m_isSending;

        static std::vector<std::string> getNames();
        Uart(const std::string& name);
        ~Uart();
        bool_t open();
        void close();
        bool_t config(uint32_t baudrate, uint8_t dataBits, Uart::Parity parity, Uart::StopBits stopBits);
        bool_t write(const uint8_t* data, uint_t size, uint32_t baudrate);
        void setRxBuffer(uint8_t* buf, uint_t size);

    private:
        uint8_t* m_rxBuf;
        uint_t m_rxBufSize;
        const uint8_t* m_txBuf;
        uint_t m_txBufSize;
        uint32_t m_txBaudrate;
        bool_t m_isSending;
        HANDLE m_portHandle;
        HANDLE m_txStartEvent;
        std::thread m_threadRxTx;
        uint32_t m_baudrate;
        uint8_t m_dataBits;
        Uart::Parity m_parity;
        Uart::StopBits m_stopBits;

        void threadRxTx();
        void resetOv(OVERLAPPED& ov);
    };
}
//--------------------------------------------------------------------------------------------------
#endif
