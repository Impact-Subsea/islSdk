#ifndef UARTPORT_H_
#define UARTPORT_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"
#include "platform/uart.h"
#include "comms/ports/sysPort.h"
#include "types/queue.h"
#include <array>

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    class UartPort : public SysPort
    {
    public:
        static std::vector<uint32_t> defaultBaudrates;
        Uart& uart = m_uart;

        UartPort(const std::string& name);
        ~UartPort();
        bool_t open() override;         ///< Open the port.
        void close() override;          ///< Close the port.
        bool_t process() override;

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

        Uart m_uart{ name };
        const uint_t m_rxBufSize = 2048;
        Queue m_tx{ 1024 * 8 };
        Queue m_rx{ 1024 * 32 };
        TxRxBuf* m_rxBuf;

        void rxDataCallback(const uint8_t* data, uint_t size, uint32_t baudrate);
        void txCompeteCallback(const uint8_t* data, uint_t bytesWritten);
        void errorCallback();

        Slot<const uint8_t*, uint_t, uint32_t> m_slotRxData {this, & UartPort::rxDataCallback};
        Slot<const uint8_t*, uint_t> m_slotTxCompete {this, & UartPort::txCompeteCallback};
    };
}
//--------------------------------------------------------------------------------------------------
#endif
