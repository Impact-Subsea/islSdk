#ifndef CONNECTIONMETA_H_
#define CONNECTIONMETA_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    /// Connection information for a port. Check the port type to know which elements are valid.
    class ConnectionMeta
    {
    public:
        ConnectionMeta(uint32_t baudrate) : baudrate(baudrate), ipAddress(0), port(0) {}
        ConnectionMeta(uint32_t ipAddress, uint16_t port) : baudrate(0), ipAddress(ipAddress), port(port) {}
        ConnectionMeta(uint32_t baudrate, uint32_t ipAddress, uint16_t port) : baudrate(baudrate), ipAddress(ipAddress), port(port) {}
        uint32_t baudrate;                          ///< Baudrate of the serial port
        uint32_t ipAddress;                         ///< IpAddress data is sent to
        uint16_t port;                              ///< The port data is sent to

        ConnectionMeta(const ConnectionMeta& meta)
        {
            baudrate = meta.baudrate;
            ipAddress = meta.ipAddress;
            port = meta.port;
        }

        bool_t isDifferent(const ConnectionMeta& meta) const
        {
            return (baudrate != meta.baudrate || ipAddress != meta.ipAddress || port != meta.port);
        }
    };
}
//--------------------------------------------------------------------------------------------------
#endif
