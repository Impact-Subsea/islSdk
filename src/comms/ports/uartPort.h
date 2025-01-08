#ifndef UARTPORT_H_
#define UARTPORT_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"
#include "platform/uart.h"
#include "comms/ports/sysPort.h"
#include "types/queue.h"
#include <array>
#include <thread>

#ifdef OS_WINDOWS
#include "platform/windows/serialPort.h"
#elif OS_UNIX
#include "platform/unix/serialPort.h"
#else
#error "Unsupported platform. Define OS_WINDOWS or OS_UNIX"
#endif

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    class UartPort : public SysPort
    {
    public:
        static std::vector<uint32_t> defaultBaudrates;
        SerialPort& serialPort = m_serialPort;

        UartPort(const std::string& name);
        ~UartPort();
        void open() override;           ///< Open the port.
        void close() override;          ///< Close the port.
        bool_t process() override;

        /**
        * @brief Configure the port.
        * @param baudrate The baudrate to configure the port to.
        * @param dataBits The number of data bits.
        * @param parity The parity to use.
        * @param stopBits The number of stop bits.
        * @return True if the port was configured successfully.
        */
        bool_t setSerial(uint32_t baudrate, uint8_t dataBits, Uart::Parity parity, Uart::StopBits stopBits);

        /**
        * @brief Write data to the port.
        * @param data The data to write.
        * @param size The size of the data to write.
        * @param meta The baudrate to send the data at.
        * @return True if the data was written successfully.
        */
        bool_t write(const uint8_t* data, uint_t size, const ConnectionMeta& meta) override;

       /**
        * @brief Discover Impact Subsea devices.
        * @param pid The product id to discover. 0xffff means any. see Device::Pid.
        * @param pn The part number to discover. 0xffff means any.
        * @param sn The serial number to discover. 0xffff means any.
        */
        void discoverIslDevices(uint16_t pid = 0xffff, uint16_t pn = 0xffff, uint16_t sn = 0xffff) override;

        /**
        * @brief Discover Impact Subsea devices.
        * @param pid The product id to discover. 0xffff means any. see Device::Pid.
        * @param pn The part number to discover. 0xffff means any.
        * @param sn The serial number to discover. 0xffff means any.
        * @param baudrate The baudrate to send the data at.
        * @param timeoutMs The timeout in milliseconds to wait for a response.
        */
        void discoverIslDevices(uint16_t pid, uint16_t pn, uint16_t sn, uint32_t baudrate, uint_t timeoutMs);

       /**
        * @brief Discover NMEA devices with the default settings.
        */
        void discoverNmeaDevices() override;

        /**
        * @brief Discover NMEA devices.
        * @param baudrate The baudrate to listen at.
        * @param timeoutMs The timeout in milliseconds to wait for a response.
        */
        void discoverNmeaDevices(uint32_t baudrate, uint_t timeoutMs);

    private:
        struct TxRxBuf
        {
            uint_t size;
            uint32_t baudrate;
            uint8_t* data;
        };

        SerialPort m_serialPort{ name };
        const uint_t m_rxBufSize = 2048;
        Queue m_tx{ 1024 * 8 };
        Queue m_rx{ 1024 * 32 };
        TxRxBuf* m_rxBuf;
        bool_t m_eventOpen;
        bool_t m_eventClose;
        std::thread m_threadOpen;
        std::thread m_threadClose;

        void rxDataCallback(const uint8_t* data, uint_t size, uint32_t baudrate);
        void txCompeteCallback(const uint8_t* data, uint_t bytesWritten);
        void uartEvent(Uart::Events event);

        Slot<const uint8_t*, uint_t, uint32_t> m_slotRxData {this, & UartPort::rxDataCallback};
        Slot<const uint8_t*, uint_t> m_slotTxCompete {this, & UartPort::txCompeteCallback};
    };
}
//--------------------------------------------------------------------------------------------------
#endif
