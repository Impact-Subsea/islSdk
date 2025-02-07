#ifndef WINDOWS_SERIALPORT_H_
#define WINDOWS_SERIALPORT_H_

//------------------------------------------ Includes ----------------------------------------------

#define WIN32_LEAN_AND_MEAN

#include "types/sdkTypes.h"
#include "platform/uart.h"
#include <string>
#include <vector>
#include <thread>
#include "windows.h"

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    class SerialPort : public Uart
    {
    public:
        
        static std::vector<std::string> getNames();
        SerialPort(const std::string& name);
        ~SerialPort();
        bool_t open() override;
        void close() override;
        bool_t isOpen() const override;
        bool_t config(uint32_t baudrate, uint8_t dataBits, Uart::Parity parity, Uart::StopBits stopBits) override;
        bool_t write(const uint8_t* data, uint_t size, uint32_t baudrate=0) override;
        void setRxBuffer(uint8_t* buf, uint_t size) override;

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
