#ifndef WINDOWS_NETSOCKET_H_
#define WINDOWS_NETSOCKET_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"
#include "types/sigSlot.h"

//--------------------------------------- Class Definition -----------------------------------------

typedef uint_t SOCKET;

namespace IslSdk
{
    class NetSocket
    {
    public:
        enum class State { Error, Ok, TcpWating, Connected, Disconnected };
        static bool_t initialise();
        static void deinitialise();
        NetSocket(bool_t isTcp, bool_t isServer, uint32_t ipAddress, uint16_t port);
        ~NetSocket();
        bool_t isOpen();
        NetSocket::State write(const uint8_t* data, uint_t* size, uint32_t ipAddress, uint16_t port);
        NetSocket::State read(uint8_t* buf, uint_t* size, uint32_t* ipAddress, uint16_t* port);

    private:
        SOCKET m_socket;
        bool_t m_isTcp;
        bool_t m_isServer;
        bool_t m_connected;
        uint32_t m_ipAddress;
        uint16_t m_port;

        SOCKET createTcpSocket(bool_t isServer, uint32_t ipAddress, uint16_t port);
        SOCKET createUdpSocket(bool_t isServer, uint32_t ipAddress, uint16_t port);
        SOCKET tcpAcceptConnection(SOCKET socket);
    };
}
//--------------------------------------------------------------------------------------------------
#endif
