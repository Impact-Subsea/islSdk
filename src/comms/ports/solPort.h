#ifndef SOLPORT_H_
#define SOLPORT_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"
#include "platform/uart.h"
#include "platform/netSocket.h"
#include "comms/ports/sysPort.h"
#include <array>

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    class SolPort : public SysPort
    {
    public:
        static std::vector<uint32_t> defaultBaudrates;

        SolPort(const std::string& name, bool_t isTcp, bool_t useTelnet, uint32_t ipAddress, uint16_t port);
        ~SolPort();
        bool_t open() override;         ///< Open the port.
        void close() override;          ///< Close the port.
        bool_t process() override;

        /**
        * @brief Configure the port.
        * @param isTcp True if the port is TCP. False if UDP.
        * @param useTelnet True if the port is telnet and uses RFC2217. False if raw.
        * @param ipAddress The ip address of the SOL (serial over LAN) adapter.
        * @param port The network port to send to.
        * @return True if the port was configured successfully.
        */
        bool_t config(bool_t isTcp, bool_t useTelnet, uint32_t ipAddress, uint16_t port);

        /**
        * @brief Configure the serial port. Only works if RFC2217 is enabled at both ends.
        * @param baudrate The baudrate to set the port to.
        * @param dataBits The number of data bits.
        * @param parity The parity.
        * @param stopBits The number of stop bits.
        * @return True if the port was configured successfully.
        */
        bool_t setSerial(uint32_t baudrate, uint8_t dataBits, Uart::Parity parity, Uart::StopBits stopBits);

        /**
        * @brief Write data to the port.
        * @param data The data to write.
        * @param size The size of the data to write.
        * @param meta The baudrate to send the data at. This is only valid if useTelnet is true.
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
        * @param baudrate The baudrate to send the data at. This is only valid if useTelnet is true.
        * @param timeoutMs The timeout in milliseconds to wait for a response.
        */
        void discoverIslDevices(uint16_t pid, uint16_t pn, uint16_t sn, uint32_t baudrate, uint_t timeoutMs);

        /**
        * @brief Discover NMEA devices with the default settings.
        */
        void discoverNmeaDevices() override;

        /**
        * @brief Discover NMEA devices.
        * @param baudrate The baudrate to listen at. This is only valid if useTelnet is true.
        * @param timeoutMs The timeout in milliseconds to wait for a response.
        */
        void discoverNmeaDevices(uint32_t baudrate, uint_t timeoutMs);

    private:
        void processTelnetCmd(uint8_t cmd, uint8_t* buf, uint_t size);
        std::unique_ptr<NetSocket> m_socket;
        bool_t m_isTcp;
        bool_t m_useTelnet;
        uint32_t m_ipAddress;
        uint16_t m_port;
        uint32_t m_baudrate;
        uint8_t m_dataBits;
        Uart::Parity m_parity;
        Uart::StopBits m_stopBits;
        uint64_t m_tcpTimeout;
        bool_t m_iac;
        struct
        {
            bool_t binary;
            bool_t comPort;
        } m_telnetModes;
        uint_t m_telnetCmdSize;
        uint8_t m_bufCmd;
        uint8_t m_rxBuf[1522];
        uint8_t m_telnetCmd[32];
        uint8_t m_txBuf[2048];
    };
}
//--------------------------------------------------------------------------------------------------
#endif
