#ifndef UNIX_SERIALPORT_H_
#define UNIX_SERIALPORT_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"
#include "platform/uart.h"
#include <string>
#include <vector>
#include <thread>

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
