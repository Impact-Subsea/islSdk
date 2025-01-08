#ifndef REMOTESERIALPORT_H_
#define REMOTESERIALPORT_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"
#include "comms/ports/sysPort.h"
#include "devices/pcpServices.h"
#include "types/queue.h"

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    class PoweredComPort : public SysPort
    {
        friend class PcpDevice;
    public:
        static std::vector<uint32_t> defaultBaudrates;

        PoweredComPort(const std::string& name, PcpServices& pcpServices);
        ~PoweredComPort();
        void open() override;           ///< Open the port.
        void close() override;          ///< Close the port.
        bool_t process() override;

        /**
        * @brief Configure the serial port.
        * @param baudrate The baudrate to set the port to.
        * @param dataBits The number of data bits.
        * @param parity The parity.
        * @param stopBits The number of stop bits.
        */
        void setSerial(uint32_t baudrate, uint8_t dataBits, Uart::Parity parity, Uart::StopBits stopBits);

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

        /**
        * @brief Set the delay between powering on the port and opening the port.
        * @param powerOnToOpenDelayMs The delay in milliseconds between power on and opening the port. A value of zero will disable auto power on.
        */
        void setAutoPowerOnOpen(uint32_t powerOnToOpenDelayMs);

        /**
        * @brief Get the id of the device which created this port. 
        * @return The device id.
        */
        uint32_t getDeviceId() const;

        /**
        * @brief Get the index of this port which the parent device assigned.
        * @return The index.
        */
        uint_t getIndex() const;

        /**
        * @brief Get the connection meta of the device which created this port.
        * @return The connection meta.
        */
        ConnectionMeta getDeviceConnectionMeta() const;

    private:
        PcpServices* m_pcpServices;
        uint32_t m_powerToDiscoveryDelayMs;
        int64_t m_powerOnTimer;

        struct RxBuf
        {
            uint_t size;
            uint32_t baudrate;
            uint8_t* data;
        };

        Queue m_rx{ 1024 * 32 };
        uint32_t m_baudrate;

        void doOpen();
        void rxData(const uint8_t* data, uint_t size, uint32_t baudrate);
    };
}
//--------------------------------------------------------------------------------------------------
#endif
