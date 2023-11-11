//------------------------------------------ Includes ----------------------------------------------

#include "netSocket.h"
#include "platform/mem.h"
#include "platform/debug.h"
#include <sys/socket.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <errno.h>

using namespace IslSdk;

const int_t SOCKET_ERROR = -1;
const int_t INVALID_SOCKET = -1;

//--------------------------------------------------------------------------------------------------
bool_t NetSocket::initialise()
{
    return true;
}
//--------------------------------------------------------------------------------------------------
void NetSocket::deinitialise()
{
}
//--------------------------------------------------------------------------------------------------
NetSocket::NetSocket(bool_t isTcp, bool_t isServer, uint32_t ipAddress, uint16_t port)
{
    if (isTcp)
    {
        m_socket = createTcpSocket(isServer, ipAddress, port);
    }
    else
    {
        m_socket = createUdpSocket(isServer, ipAddress, port);
    }

    m_isTcp = isTcp;
    m_isServer = isServer;
    m_connected = false;
    m_ipAddress = ipAddress;
    m_port = port;
}
//--------------------------------------------------------------------------------------------------
NetSocket::~NetSocket()
{
    shutdown(m_socket, SHUT_WR);
    close(m_socket);
}
//--------------------------------------------------------------------------------------------------
bool_t NetSocket::isOpen()
{
    return m_socket >= 0;
}
//--------------------------------------------------------------------------------------------------
NetSocket::State NetSocket::write(const uint8_t* data, uint_t* size, uint32_t ipAddress, uint16_t port)
{
    NetSocket::State socketStatus = State::Ok;
    fd_set waitSend;
    struct timeval timeout;

    FD_ZERO(&waitSend);
    FD_SET(m_socket, &waitSend);
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    int_t result = select(m_socket + 1, NULL, &waitSend, NULL, &timeout);
    if (result != SOCKET_ERROR)
    {
        if (FD_ISSET(m_socket, &waitSend))
        {
            struct sockaddr_in addr;
            addr.sin_family = AF_INET;
            addr.sin_port = htons(port);
            addr.sin_addr.s_addr = ipAddress;

            result = sendto(m_socket, reinterpret_cast<const char*>(data), static_cast<int>(*size), 0, (struct sockaddr*)&addr, sizeof(addr));
            *size = result;
        }
        else
        {
            if (m_isTcp)
            {
                socketStatus = State::TcpWating;
            }
            *size = 0;
        }
    }

    if (result == SOCKET_ERROR)
    {
        socketStatus = State::Error;
        *size = 0;
        debugLog("NetPort", "network write falied with error %i", FMT_I(errno));
    }

    return socketStatus;
}
//--------------------------------------------------------------------------------------------------
NetSocket::State NetSocket::read(uint8_t* buf, uint_t* size, uint32_t* ipAddress, uint16_t* port)
{
    NetSocket::State socketStatus = NetSocket::State::Ok;
    fd_set waitRecv;
    struct timeval timeout;

    FD_ZERO(&waitRecv);
    FD_SET(m_socket, &waitRecv);
    FD_SET(STDIN_FILENO, &waitRecv);
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;
    *ipAddress = 0;
    *port = 0;
    int_t bytesRead = 0;

    int_t result = select(m_socket + 1, &waitRecv, NULL, NULL, &timeout);
    if (result != SOCKET_ERROR)
    {
        if (FD_ISSET(m_socket, &waitRecv))
        {
            if (m_isTcp && m_isServer && !m_connected)
            {
                m_socket = tcpAcceptConnection(m_socket);
                m_connected = m_socket != INVALID_SOCKET;
                if (m_connected)
                {
                    socketStatus = State::Connected;
                }
            }
            else
            {
                struct sockaddr_in fromAddress;
                uint fromAddressSize = sizeof(fromAddress);
                result = recvfrom(m_socket, reinterpret_cast<char*>(buf), static_cast<int>(*size), 0, (struct sockaddr*)&fromAddress, &fromAddressSize);
                if (result != SOCKET_ERROR)
                {
                    *ipAddress = fromAddress.sin_addr.s_addr;
                    *port = htons(fromAddress.sin_port);
                    bytesRead = result;
                    m_connected = result != 0;

                    if (m_isTcp && !m_connected)
                    {
                        socketStatus = State::Disconnected;
                        shutdown(m_socket, SHUT_WR);
                        close(m_socket);
                        if (m_isServer)
                        {
                            result = (int_t)createTcpSocket(m_isServer, m_ipAddress, m_port);
                            m_socket = result;
                            socketStatus = State::TcpWating;
                        }
                    }
                }
            }
        }
    }

    *size = bytesRead;

    if (result == SOCKET_ERROR)
    {
        socketStatus = State::Error;
        debugLog("NetPort", "network read falied with error %i", FMT_I(errno));
    }

    return socketStatus;
}
//--------------------------------------------------------------------------------------------------
SOCKET NetSocket::createTcpSocket(bool_t isServer, uint32_t ipAddress, uint16_t port)
{
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = ipAddress;
    addr.sin_port = htons(port);

    SOCKET sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock == INVALID_SOCKET)
    {
        debugLog("NetPort", "socket falied to create %i", FMT_I(errno));
        return INVALID_SOCKET;
    }

    int_t flags = fcntl(sock, F_GETFL);
    if (flags == -1)
    {
        debugLog("NetPort", "Could not get socket flags");
        close(sock);
        return INVALID_SOCKET;
    }

    if (fcntl(sock, F_SETFL, flags | O_NONBLOCK) == -1)
    {
        debugLog("NetPort", "Could not set socket to non-blocking mode");
        close(sock);
        return INVALID_SOCKET;
    }

    if (isServer)
    {
        if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) == SOCKET_ERROR)
        {
            debugLog("NetPort", "bind falied with error %i", FMT_I(errno));
            close(sock);
            return INVALID_SOCKET;
        }

        if (listen(sock, 1) == SOCKET_ERROR)
        {
            debugLog("NetPort", "listen falied with error %i", FMT_I(errno));
            close(sock);
            return INVALID_SOCKET;
        }
    }
    else
    {
        if (connect(sock, (struct sockaddr*)&addr, sizeof(addr)) == SOCKET_ERROR)
        {
            if (errno != EWOULDBLOCK)
            {
                debugLog("NetPort", "connect falied with error %i", FMT_I(errno));
                close(sock);
                return INVALID_SOCKET;
            }
        }
    }

    return sock;
}
//--------------------------------------------------------------------------------------------------
SOCKET NetSocket::createUdpSocket(bool_t isServer, uint32_t ipAddress, uint16_t port)
{
    SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == INVALID_SOCKET)
    {
        debugLog("NetPort", "socket falied to create %d", errno);
        return INVALID_SOCKET;
    }

    int_t flags = fcntl(sock, F_GETFL);
    if (flags == -1)
    {
        debugLog("NetPort", "Could not get socket flags");
        close(sock);
        return INVALID_SOCKET;
    }

    if (fcntl(sock, F_SETFL, flags | O_NONBLOCK) == -1)
    {
        debugLog("NetPort", "Cound not set socket to non-blocking mode");
        close(sock);
        return INVALID_SOCKET;
    }

    int32_t rbufsize = 1024 * 512;
    if (setsockopt(sock, SOL_SOCKET, SO_RCVBUF, (char*)&rbufsize, sizeof(rbufsize)) == SOCKET_ERROR)
    {
        debugLog("NetPort", "Cound not set socket rx buffer size");
    }

    if (isServer)
    {
        struct sockaddr_in addr;
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = ipAddress;
        addr.sin_port = htons(port);

        if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) == SOCKET_ERROR)
        {
            debugLog("NetPort", "bind falied with error %i", FMT_I(errno));
            close(sock);
            return INVALID_SOCKET;
        }
    }
    else
    {
        int32_t iOptVal = 1;
        if (setsockopt(sock, SOL_SOCKET, SO_BROADCAST, (char*)&iOptVal, sizeof(iOptVal)) == SOCKET_ERROR)
        {
            debugLog("NetPort", "setsockopt falied with error %i", FMT_I(errno));
            close(sock);
            return INVALID_SOCKET;
        }
    }

    return sock;
}
//--------------------------------------------------------------------------------------------------
SOCKET NetSocket::tcpAcceptConnection(SOCKET socket)
{
    SOCKET newSocket = accept(socket, NULL, NULL);

    if (newSocket != INVALID_SOCKET)
    {
        shutdown(socket, SHUT_RDWR);
        close(socket);

        int_t flags = fcntl(newSocket, F_GETFL);
        if (flags == -1)
        {
            debugLog("NetPort", "Could not get socket flags");
            close(newSocket);
            newSocket = INVALID_SOCKET;
        }

        if (fcntl(socket, F_SETFL, flags | O_NONBLOCK) == -1)
        {
            debugLog("NetPort", "Cund not set socket to non-blocking mode");
            close(newSocket);
            newSocket = INVALID_SOCKET;
        }
    }

    return newSocket;
}
//--------------------------------------------------------------------------------------------------
