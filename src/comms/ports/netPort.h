#ifndef NETPORT_H_
#define NETPORT_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"
#include "platform/netSocket.h"
#include "comms/ports/sysPort.h"

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    class NetPort : public SysPort
    {
    public:
        static const uint16_t defaultPort = 33005;

        NetPort(const std::string& name, bool_t isTcp, bool_t isServer, uint32_t ipAddress, uint16_t port);
        ~NetPort();
        bool_t open() override;                        ///< Open the port.
        void close() override;                         ///< Close the port.
        bool_t process() override;

        /**
        * @brief Write data to the port.
        * @param data The data to write.
        * @param size The size of the data to write.
        * @param meta The ip address and port to send to.
        * @return True if the data was written successfully.
        */
        bool_t write(const uint8_t* data, uint_t size, const ConnectionMeta& meta) override;

        /**
        * @brief discover Impact Subsea devices.
        * @param pid The product id to discover. 0xffff means any. see Device::Pid.
        * @param pn The part number to discover. 0xffff means any.
        * @param sn The serial number to discover. 0xffff means any.
        */
        void discoverIslDevices(uint16_t pid = 0xffff, uint16_t pn  = 0xffff, uint16_t sn = 0xffff) override;

        /**
        * @brief discover Impact Subsea devices.
        * @param pid The product id to discover. 0xffff means any. see Device::Pid.
        * @param pn The part number to discover. 0xffff means any.
        * @param sn The serial number to discover. 0xffff means any.
        * @param ipAddress The ip address to send the discovery to.
        * @param port The port to send the discovery to.
        * @param timeoutMs The timeout in milliseconds to wait for a response.
        */
        void discoverIslDevices(uint16_t pid, uint16_t pn, uint16_t sn, uint32_t ipAddress, uint16_t port, uint_t timeoutMs);

    private:
        std::unique_ptr<NetSocket> m_socket;
        bool_t m_isTcp;
        bool_t m_isServer;
        uint32_t m_ipAddress;
        uint16_t m_port;
    };
}
//--------------------------------------------------------------------------------------------------
#endif
