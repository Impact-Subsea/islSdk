//------------------------------------------ Includes ----------------------------------------------

#include "solPort.h"
#include "platform/timeUtils.h"
#include "platform/debug.h"

using namespace IslSdk;

enum TelnetCmd : uint8_t { Se = 0xf0, Sb = 0xfa, Will = 0xfb, Wont = 0xfc, Do = 0xfd, Dont = 0xfe, Iac = 0xff };
enum ComPortCmd : uint8_t { TransmitBinary = 0, Options = 44, Baudrate = 1, DataBits = 2, Parity = 3, StopBits = 4, Request = 100 };

std::vector<uint32_t> SolPort::defaultBaudrates = { 115200, 9600, 57600, 38400, 19200, 4800 };

//--------------------------------------------------------------------------------------------------
SolPort::SolPort(const std::string& name, bool_t isTcp, bool_t useTelnet, uint32_t ipAddress, uint16_t port) :
    SysPort(name, ClassType::Sol, Type::Serial, 500),
    m_isTcp(isTcp),
    m_useTelnet(useTelnet),
    m_ipAddress(ipAddress),
    m_port(port),
    m_baudrate(9600),
    m_dataBits(8),
    m_parity(Uart::Parity::None),
    m_stopBits(Uart::StopBits::One),
    m_tcpTimeout(0),
    m_iac(false),
    m_telnetModes{ false, false },
    m_bufCmd(0),
    m_telnetCmdSize(0),
    m_rxBuf{},
    m_telnetCmd{},
    m_txBuf{}
{
}
//--------------------------------------------------------------------------------------------------
SolPort::~SolPort()
{
    close();
}
//--------------------------------------------------------------------------------------------------
void SolPort::open()
{
    if (!m_isOpen)
    {
        m_socket = std::make_unique<NetSocket>(m_isTcp, false, m_ipAddress, m_port);
        m_isOpen = m_socket->isOpen();
        m_active = m_isOpen;
        setSerial(m_baudrate, m_dataBits, m_parity, m_stopBits);
        debugLog("SysPort", "%s opening", name.c_str());
        onOpen(*this, m_isOpen);
    }
}
//--------------------------------------------------------------------------------------------------
void SolPort::close()
{
    m_tcpTimeout = 0;
    m_iac = false;
    m_telnetModes.binary = false;
    m_telnetModes.comPort = false;
    m_bufCmd = 0;
    m_telnetCmdSize = 0;

    if (m_socket)
    {
        m_socket.reset();
    }
    SysPort::close();
}
//--------------------------------------------------------------------------------------------------
bool_t SolPort::config(bool_t isTcp, bool_t useTelnet, uint32_t ipAddress, uint16_t port)
{
    m_isTcp = isTcp;
    m_useTelnet = useTelnet;
    m_ipAddress = ipAddress;
    m_port = port;

    if (m_isOpen)
    {
        close();
        open();
    }

    return true;
}
//--------------------------------------------------------------------------------------------------
bool_t SolPort::setSerial(uint32_t baudrate, uint8_t dataBits, Uart::Parity parity, Uart::StopBits stopBits)
{
    const uint8_t stopBitsLut[] = { 1, 3, 2 };
    bool_t ok = true;

    if (m_useTelnet && m_isOpen)
    {
        uint_t i = 0;
        uint8_t cmd[64];
        cmd[i++] = TelnetCmd::Iac;
        cmd[i++] = TelnetCmd::Sb;
        cmd[i++] = ComPortCmd::Options;
        cmd[i++] = ComPortCmd::Baudrate;
        cmd[i++] = static_cast<uint8_t>(baudrate >> 24);
        cmd[i++] = static_cast<uint8_t>(baudrate >> 16);
        cmd[i++] = static_cast<uint8_t>(baudrate >> 8);
        cmd[i++] = static_cast<uint8_t>(baudrate);
        cmd[i++] = TelnetCmd::Iac;
        cmd[i++] = TelnetCmd::Se;

        cmd[i++] = TelnetCmd::Iac;
        cmd[i++] = TelnetCmd::Sb;
        cmd[i++] = ComPortCmd::Options;
        cmd[i++] = ComPortCmd::DataBits;
        cmd[i++] = dataBits;
        cmd[i++] = TelnetCmd::Iac;
        cmd[i++] = TelnetCmd::Se;

        cmd[i++] = TelnetCmd::Iac;
        cmd[i++] = TelnetCmd::Sb;
        cmd[i++] = ComPortCmd::Options;
        cmd[i++] = ComPortCmd::Parity;
        cmd[i++] = static_cast<uint8_t>(parity) + 1;
        cmd[i++] = TelnetCmd::Iac;
        cmd[i++] = TelnetCmd::Se;

        cmd[i++] = TelnetCmd::Iac;
        cmd[i++] = TelnetCmd::Sb;
        cmd[i++] = ComPortCmd::Options;
        cmd[i++] = ComPortCmd::StopBits;
        cmd[i++] = stopBitsLut[static_cast<uint_t>(stopBits)];
        cmd[i++] = TelnetCmd::Iac;
        cmd[i++] = TelnetCmd::Se;

        ok = m_socket->write(&cmd[0], &i, m_ipAddress, m_port) == NetSocket::State::Ok;
    }

    if (ok)
    {
        m_baudrate = baudrate;
        m_parity = parity;
        m_dataBits = dataBits;
        m_stopBits = stopBits;
    }

    return ok;
}
//--------------------------------------------------------------------------------------------------
bool_t SolPort::write(const uint8_t* data, uint_t size, const ConnectionMeta& meta)
{
    bool_t written = false;

    if (!m_lock && m_socket)
    {
        if (meta.baudrate && m_baudrate != meta.baudrate)
        {
            setSerial(meta.baudrate, m_dataBits, m_parity, m_stopBits);
        }

        if (m_codec && data && size)
        {
            size = m_codec->encode(data, size, &m_txBuf[0], sizeof(m_txBuf));
            data = &m_txBuf[0];
        }

        if (m_useTelnet && data)
        {
            uint_t byteCount = 0;

            for (uint_t i = 0; i < size; i++)
            {
                if (*data == 0xff)
                {
                    m_rxBuf[byteCount++] = 0xff;
                }

                m_rxBuf[byteCount++] = *data++;
            }

            size = byteCount;
            data = &m_rxBuf[0];
        }

        uint_t bytesSent = 0;

        if (data)
        {
            while (bytesSent < size)
            {
                uint_t amountToSend = (size - bytesSent) > 1200 ? 1200 : (size - bytesSent);
                NetSocket::State state = m_socket->write(&data[bytesSent], &amountToSend, m_ipAddress, m_port);
                if (state == NetSocket::State::Ok)
                {
                    bytesSent += amountToSend;
                }
                else
                {
                    if (state == NetSocket::State::Error)
                    {
                        m_portError = true;
                    }
                    else if (state == NetSocket::State::TcpWating)
                    {
                        if (m_tcpTimeout == 0)
                        {
                            m_tcpTimeout = 5000 + Time::getTimeMs();
                        }
                        else if (Time::getTimeMs() >= m_tcpTimeout)
                        {
                            m_tcpTimeout = 0;
                            m_portError = true;
                        }
                    }
                    break;
                }
            }
        }

        if (bytesSent == size)
        {
            written = true;
            txComplete(data, size);
        }
    }

    return written;
}
//--------------------------------------------------------------------------------------------------
void SolPort::discoverIslDevices(uint16_t pid, uint16_t pn, uint16_t sn)
{
    for (uint_t i = 0; i < defaultBaudrates.size(); i++)
    {
        SysPort::discoverIslDevices(pid, pn, sn, ConnectionMeta(defaultBaudrates[i]), discoveryTimeoutMs, 1);
    }
}
//--------------------------------------------------------------------------------------------------
void SolPort::discoverIslDevices(uint16_t pid, uint16_t pn, uint16_t sn, uint32_t baudrate, uint_t timeoutMs)
{
    SysPort::discoverIslDevices(pid, pn, sn, ConnectionMeta(baudrate), timeoutMs, 1);
}
//--------------------------------------------------------------------------------------------------
void SolPort::discoverNmeaDevices()
{
    for (uint_t i = 0; i < defaultBaudrates.size(); i++)
    {
        nemaDiscovery(ConnectionMeta(defaultBaudrates[i]), 1500);
    }
}
//--------------------------------------------------------------------------------------------------
void SolPort::discoverNmeaDevices(uint32_t baudrate, uint_t timeoutMs)
{
    nemaDiscovery(ConnectionMeta(baudrate), timeoutMs);
}
//--------------------------------------------------------------------------------------------------
bool_t SolPort::process()
{
    uint32_t fromAddress;
    uint16_t remoteSrcPort;

    if (m_socket)
    {
        uint_t size = sizeof(m_rxBuf);
        NetSocket::State state = m_socket->read(&m_rxBuf[0], &size, &fromAddress, &remoteSrcPort);

        while (state == NetSocket::State::Ok && size)
        {
            m_rxBytesCount += size;

            ConstBuffer buf = { &m_rxBuf[0], size };
            onRxData(*this, buf);

            if (m_useTelnet)
            {
                uint_t byteCount = 0;

                for (uint_t i = 0; i < size; i++)
                {
                    if (m_rxBuf[i] == 0xff && !m_iac)
                    {
                        m_iac = true;
                        continue;
                    }

                    if (m_iac)
                    {
                        m_iac = false;

                        switch (m_rxBuf[i])
                        {
                        case 0xff:
                            break;

                        case 0xf0:
                            m_bufCmd = 1;
                            break;

                        case 0xfa:
                            m_telnetCmdSize = 0;
                            m_bufCmd = 33;
                            break;

                        default:
                            m_telnetCmdSize = 0;
                            m_bufCmd = 2;
                            break;
                        }
                    }

                    if (m_bufCmd)
                    {
                        m_telnetCmd[m_telnetCmdSize++] = m_rxBuf[i];
                        m_bufCmd--;
                        if (m_bufCmd == 0)
                        {
                            processTelnetCmd(m_telnetCmd[0], &m_telnetCmd[1], m_telnetCmdSize - 1);
                        }
                    }
                    else
                    {
                        m_rxBuf[byteCount++] = m_rxBuf[i];
                    }
                }
                size = byteCount;
            }

            if (m_codec)
            {
                uint_t bytesToProcess = size;

                while (bytesToProcess && m_codec)
                {
                    uint_t frameSize = m_codec->decode(&m_rxBuf[size - bytesToProcess], &bytesToProcess);

                    if (frameSize)
                    {
                        newFrameEvent(*this, &m_codec->m_frameBuf[0], frameSize, ConnectionMeta(fromAddress, remoteSrcPort), m_codec->type);
                    }
                }
            }
            else
            {
                newFrameEvent(*this, &m_rxBuf[0], size, ConnectionMeta(fromAddress, remoteSrcPort), Codec::Type::None);
            }

            if (!m_socket)
            {
                break;
            }
            size = sizeof(m_rxBuf);
            state = m_socket->read(&m_rxBuf[0], &size, &fromAddress, &remoteSrcPort);
        }

        if (state == NetSocket::State::Error)
        {
            m_portError = true;
        }
    }

    return SysPort::process();
}
//--------------------------------------------------------------------------------------------------
void SolPort::processTelnetCmd(uint8_t cmd, uint8_t* buf, uint_t size)
{
    uint8_t msg[16];
    uint_t i = 3;

    msg[0] = TelnetCmd::Iac;

    switch (cmd)
    {
    case TelnetCmd::Will:
        debugLog("SolPort", "TELNET WILL %u", FMT_U(buf[0]));

        if (buf[0] == ComPortCmd::TransmitBinary && !m_telnetModes.binary)
        {
            msg[1] = TelnetCmd::Do;
            m_telnetModes.binary = true;
            m_socket->write(&msg[0], &i, m_ipAddress, m_port);
        }
        else if (buf[0] == ComPortCmd::Options && !m_telnetModes.comPort)
        {
            msg[1] = TelnetCmd::Do;
            m_telnetModes.comPort = true;
            m_socket->write(&msg[0], &i, m_ipAddress, m_port);
        }
        else
        {
            msg[1] = TelnetCmd::Dont;
            m_socket->write(&msg[0], &i, m_ipAddress, m_port);
        }
        break;

    case TelnetCmd::Wont:
        break;

    case TelnetCmd::Do:
        debugLog("SolPort", "TELNET DO %u", FMT_U(buf[0]));
        if (buf[0] == ComPortCmd::TransmitBinary)
        {
            msg[1] = TelnetCmd::Will;
            m_telnetModes.binary = true;
            m_socket->write(&msg[0], &i, m_ipAddress, m_port);
        }
        else if (buf[0] == ComPortCmd::Options)
        {
            msg[1] = TelnetCmd::Will;
            m_telnetModes.comPort = true;
            m_socket->write(&msg[0], &i, m_ipAddress, m_port);
        }
        else
        {
            msg[1] = TelnetCmd::Wont;
            m_socket->write(&msg[0], &i, m_ipAddress, m_port);
        }
        break;

    case TelnetCmd::Dont:
        break;

    case TelnetCmd::Sb:
        if (buf[0] == ComPortCmd::Options)
        {
            debugLog("SolPort", "TELENT SB %u %u", FMT_U(buf[0]), FMT_U(buf[1]));
            switch (buf[1])
            {
            case ComPortCmd::Request + ComPortCmd::Baudrate:
                msg[1] = TelnetCmd::Sb;
                msg[2] = ComPortCmd::Options;
                msg[3] = (m_baudrate >> 24);
                msg[4] = (m_baudrate >> 16);
                msg[5] = (m_baudrate >> 8);
                msg[6] = m_baudrate;
                msg[7] = TelnetCmd::Iac;
                msg[8] = TelnetCmd::Se;
                i = 9;
                m_socket->write(&msg[0], &i, m_ipAddress, m_port);
                break;

            case ComPortCmd::Request + ComPortCmd::DataBits:
                msg[1] = TelnetCmd::Sb;
                msg[2] = ComPortCmd::Options;
                msg[3] = m_dataBits;
                msg[4] = TelnetCmd::Iac;
                msg[5] = TelnetCmd::Se;
                i = 6;
                m_socket->write(&msg[0], &i, m_ipAddress, m_port);
                break;

            case ComPortCmd::Request + ComPortCmd::Parity:
                msg[1] = TelnetCmd::Sb;
                msg[2] = ComPortCmd::Options;
                msg[3] = static_cast<uint8_t>(m_parity) + 1;
                msg[4] = TelnetCmd::Iac;
                msg[5] = TelnetCmd::Se;
                i = 6;
                m_socket->write(&msg[0], &i, m_ipAddress, m_port);
                break;

            case ComPortCmd::Request + ComPortCmd::StopBits:
                msg[1] = TelnetCmd::Sb;
                msg[2] = ComPortCmd::Options;
                msg[3] = static_cast<uint8_t>(m_stopBits) + 1;
                msg[4] = TelnetCmd::Iac;
                msg[5] = TelnetCmd::Se;
                i = 6;
                m_socket->write(&msg[0], &i, m_ipAddress, m_port);
                break;
            }
        }
        break;

    default:
        break;
    }
}
//--------------------------------------------------------------------------------------------------
