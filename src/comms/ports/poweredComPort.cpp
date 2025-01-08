//------------------------------------------ Includes ----------------------------------------------

#include "poweredComPort.h"
#include "platform/mem.h"
#include "platform/debug.h"
#include "platform/timeUtils.h"

using namespace IslSdk;

std::vector<uint32_t> PoweredComPort::defaultBaudrates = { 115200, 9600, 57600, 38400, 19200, 4800 };

//--------------------------------------------------------------------------------------------------
PoweredComPort::PoweredComPort(const std::string& name, PcpServices& pcpServices) : SysPort(name, ClassType::Pcp, Type::Serial, 500), 
m_pcpServices(&pcpServices), 
m_baudrate(0), 
m_powerToDiscoveryDelayMs(0),
m_powerOnTimer(0)
{
}
//--------------------------------------------------------------------------------------------------
PoweredComPort::~PoweredComPort()
{
    close();
}
//--------------------------------------------------------------------------------------------------
void PoweredComPort::open()
{
    if (!m_isOpen && m_powerOnTimer == 0)
    {   
        m_active = true;
        if (m_powerToDiscoveryDelayMs && m_pcpServices)
		{
            m_pcpServices->setPower(true);
            m_powerOnTimer = Time::getSystemTimeMs() + m_powerToDiscoveryDelayMs;
		}
        else
        {
            doOpen();
		}
    }
}
//--------------------------------------------------------------------------------------------------
void PoweredComPort::doOpen()
{
    m_rx.reset();
    m_baudrate = 0;
    if (m_pcpServices)
    {
        m_pcpServices->open();
    }
    m_isOpen = true;
    debugLog("SysPort", "%s opening", name.c_str());
    onOpen(*this, m_isOpen);
}
//--------------------------------------------------------------------------------------------------
void PoweredComPort::close()
{
    if (m_isOpen)
    {
        if (m_pcpServices)
        {
            m_pcpServices->close();
            if (m_powerToDiscoveryDelayMs)
			{
				m_pcpServices->setPower(false);
			}
        }
        SysPort::close();
    }
}
//--------------------------------------------------------------------------------------------------
void PoweredComPort::setSerial(uint32_t baudrate, uint8_t dataBits, Uart::Parity parity, Uart::StopBits stopBits)
{
    m_baudrate = baudrate;
    if (m_pcpServices)
    {
        m_pcpServices->setSerial(baudrate, dataBits, parity, stopBits);
    }
}
//--------------------------------------------------------------------------------------------------
bool_t PoweredComPort::write(const uint8_t* data, uint_t size, const ConnectionMeta& meta)
{
    bool_t written = false;
    uint8_t txBuf[1300];

    if (!m_lock)
    {
        if (meta.baudrate && m_baudrate != meta.baudrate)
        {
            m_baudrate = meta.baudrate;
            if (m_pcpServices)
            {
                m_pcpServices->setSerial(meta.baudrate);
            }
        }

        if (m_codec && data && size)
        {
            size = m_codec->encode(data, size, &txBuf[0], sizeof(txBuf));
            data = &txBuf[0];
        }

        if (m_pcpServices)
        {
            written = m_pcpServices->write(data, size);
        }
        if (written)
        {
            txComplete(data, size);
        }
    }

    return written;
}
//--------------------------------------------------------------------------------------------------
void PoweredComPort::discoverIslDevices(uint16_t pid, uint16_t pn, uint16_t sn)
{
    for (uint_t i = 0; i < defaultBaudrates.size(); i++)
    {
        SysPort::discoverIslDevices(pid, pn, sn, ConnectionMeta(defaultBaudrates[i]), discoveryTimeoutMs, 1);
    }
}
//--------------------------------------------------------------------------------------------------
void PoweredComPort::discoverIslDevices(uint16_t pid, uint16_t pn, uint16_t sn, uint32_t baudrate, uint_t timeoutMs)
{
    SysPort::discoverIslDevices(pid, pn, sn, ConnectionMeta(baudrate), timeoutMs, 1);
}
//--------------------------------------------------------------------------------------------------
void PoweredComPort::discoverNmeaDevices()
{
    for (uint_t i = 0; i < defaultBaudrates.size(); i++)
    {
        nemaDiscovery(ConnectionMeta(defaultBaudrates[i]), 1500);
    }
}
//--------------------------------------------------------------------------------------------------
void PoweredComPort::discoverNmeaDevices(uint32_t baudrate, uint_t timeoutMs)
{
    nemaDiscovery(ConnectionMeta(baudrate), timeoutMs);
}
//--------------------------------------------------------------------------------------------------
void PoweredComPort::setAutoPowerOnOpen(uint32_t powerOnToOpenDelayMs)
{
	m_powerToDiscoveryDelayMs = powerOnToOpenDelayMs;
}
//--------------------------------------------------------------------------------------------------
uint32_t PoweredComPort::getDeviceId() const
{
    return m_pcpServices ? m_pcpServices->getDeviceId() : 0;
}
//--------------------------------------------------------------------------------------------------
uint_t PoweredComPort::getIndex() const
{
    return m_pcpServices ? m_pcpServices->getIndex() : 0;
}
//--------------------------------------------------------------------------------------------------
ConnectionMeta PoweredComPort::getDeviceConnectionMeta() const
{
    return m_pcpServices ? m_pcpServices->getDeviceConnectionMeta() : ConnectionMeta(0);
}
//--------------------------------------------------------------------------------------------------
bool_t PoweredComPort::process()
{
    if (m_powerOnTimer)
	{
        if (Time::getSystemTimeMs() >= m_powerOnTimer)
        {
            m_powerOnTimer = 0;
            doOpen();
        }
	}

    while (RxBuf* buf = reinterpret_cast<RxBuf*>(m_rx.peekNextItem()))
    {
        m_rxBytesCount += buf->size;
        ConstBuffer buffer = { buf->data, buf->size };
        onRxData(*this, buffer);

        if (m_codec)
        {
            uint_t bytesToProcess = buf->size;

            while (bytesToProcess && m_codec)
            {
                uint_t frameSize = m_codec->decode(&buf->data[buf->size - bytesToProcess], &bytesToProcess);

                if (frameSize)
                {
                    newFrameEvent(*this, &m_codec->m_frameBuf[0], frameSize, ConnectionMeta(buf->baudrate), m_codec->type);
                }
            }
        }
        else
        {
            newFrameEvent(*this, buf->data, buf->size, ConnectionMeta(buf->baudrate), Codec::Type::None);
        }

        m_rx.pop();
    }

    return SysPort::process();
}
//--------------------------------------------------------------------------------------------------
void PoweredComPort::rxData(const uint8_t* data, uint_t size, uint32_t baudrate)
{
    RxBuf* buf = reinterpret_cast<RxBuf*>(m_rx.newItem(sizeof(RxBuf) + size));
    if (buf)
    {
        buf->data = (uint8_t*)buf + sizeof(RxBuf);
        buf->size = size;
        buf->baudrate = baudrate;
        Mem::memcpy(buf->data, data, size);
        m_rx.push();
    }
}
//--------------------------------------------------------------------------------------------------

