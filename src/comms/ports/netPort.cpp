//------------------------------------------ Includes ----------------------------------------------

#include "netPort.h"
#include "utils/utils.h"
#include "platform/debug.h"

using namespace IslSdk;

//--------------------------------------------------------------------------------------------------
NetPort::NetPort(const std::string& name, bool_t isTcp, bool_t isServer, uint32_t ipAddress, uint16_t port) :
    SysPort(name, Type::Net, 1000),
    m_isTcp(isTcp),
    m_isServer(isServer),
    m_ipAddress(ipAddress),
    m_port(port)
{
}
//--------------------------------------------------------------------------------------------------
NetPort::~NetPort()
{
}
//--------------------------------------------------------------------------------------------------
bool_t NetPort::open()
{
    if (!m_isOpen)
    {
        m_socket = std::make_unique<NetSocket>(m_isTcp, m_isServer, m_ipAddress, m_port);
        m_isOpen = m_socket->isOpen();
        debugLog("SysPort", "%s opening", name.c_str());
        onOpen(*this, !m_isOpen);
    }
    return m_isOpen;
}
//--------------------------------------------------------------------------------------------------
void NetPort::close()
{
    if (m_socket)
    {
        m_socket.reset();
        SysPort::close();
    }
}
//--------------------------------------------------------------------------------------------------
bool_t NetPort::write(const uint8_t* data, uint_t size, const ConnectionMeta& meta)
{
    bool_t written = false;

    if (m_socket)
    {
        NetSocket::State state = m_socket->write(data, &size, meta.ipAddress, meta.port);

        if (state == NetSocket::State::Ok)
        {
            written = true;
            txComplete(data, size);
        }
        else if (state == NetSocket::State::Error)
        {
            m_portError = true;
        }
    }

    return written;
}
//--------------------------------------------------------------------------------------------------
void NetPort::discoverIslDevices(uint16_t pid, uint16_t pn, uint16_t sn)
{
    SysPort::discoverIslDevices(pid, pn, sn, ConnectionMeta(Utils::ipToUint(255, 255, 255, 255), defaultPort), discoveryTimeoutMs, 1);
}
//--------------------------------------------------------------------------------------------------
void NetPort::discoverIslDevices(uint16_t pid, uint16_t pn, uint16_t sn, uint32_t ipAddress, uint16_t port, uint_t timeoutMs)
{
    SysPort::discoverIslDevices(pid, pn, sn, ConnectionMeta(ipAddress, port), timeoutMs, 1);
}
//--------------------------------------------------------------------------------------------------
bool_t NetPort::process()
{
    uint32_t fromAddress;
    uint16_t remoteSrcPort;
    uint8_t rxBuf[1522];

    if (m_socket)
    {
        uint_t size = sizeof(rxBuf);
        NetSocket::State state = m_socket->read(&rxBuf[0], &size, &fromAddress, &remoteSrcPort);

        while (state == NetSocket::State::Ok && size)
        {
            m_rxBytesCount += size;

            onRxData(*this, &rxBuf[0], size);

            if (m_codec)
            {
                uint_t bytesToProcess = size;

                while (bytesToProcess)
                {
                    uint_t frameSize = m_codec->decode(&rxBuf[size - bytesToProcess], &bytesToProcess);

                    if (frameSize)
                    {
                        newFrameEvent(*this, &m_codec->m_frameBuf[0], frameSize, ConnectionMeta(fromAddress, remoteSrcPort), m_codec->type);
                    }
                }
            }
            else
            {
                newFrameEvent(*this, &rxBuf[0], size, ConnectionMeta(fromAddress, remoteSrcPort), Codec::Type::None);
            }

            size = sizeof(rxBuf);
            state = m_socket->read(&rxBuf[0], &size, &fromAddress, &remoteSrcPort);
        }

        if (state == NetSocket::State::Error)
        {
            m_portError = true;
        }
    }

    return SysPort::process();
}
//--------------------------------------------------------------------------------------------------
