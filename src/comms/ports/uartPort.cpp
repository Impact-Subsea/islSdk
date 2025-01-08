//------------------------------------------ Includes ----------------------------------------------

#include "uartPort.h"
#include "platform/mem.h"
#include "platform/debug.h"

using namespace IslSdk;

std::vector<uint32_t> UartPort::defaultBaudrates = { 115200, 9600, 57600, 38400, 19200, 4800 };

//--------------------------------------------------------------------------------------------------
UartPort::UartPort(const std::string& name) : SysPort(name, ClassType::Serial, Type::Serial, 250)
{
    m_rxBuf = nullptr;
    m_eventOpen = false;
    m_eventClose = false;
    m_serialPort.rxDataEvent.connect(m_slotRxData);
    m_serialPort.txCompleteEvent.connect(m_slotTxCompete);
    m_serialPort.uartEvent.connect(this, &UartPort::uartEvent);
}
//--------------------------------------------------------------------------------------------------
UartPort::~UartPort()
{
    if (m_threadOpen.joinable())
	{
		m_threadOpen.join();
	}

    close();

    if (m_threadClose.joinable())
    {
        m_threadClose.join();
    }
}
//--------------------------------------------------------------------------------------------------
void UartPort::open()
{
    if (!m_isOpen && !m_threadOpen.joinable())
    {
        m_rx.reset();
        m_tx.reset();
        m_rxBuf = nullptr;
        m_active = true;
        m_threadOpen = std::thread(&SerialPort::open, &m_serialPort);
    }
}
//--------------------------------------------------------------------------------------------------
void UartPort::close()
{
    if (m_isOpen && !m_threadClose.joinable())
    {
        m_threadClose = std::thread(&SerialPort::close, &m_serialPort);
    }
}
//--------------------------------------------------------------------------------------------------
bool_t UartPort::setSerial(uint32_t baudrate, uint8_t dataBits, Uart::Parity parity, Uart::StopBits stopBits)
{
	return m_serialPort.config(baudrate, dataBits, parity, stopBits);
}
//--------------------------------------------------------------------------------------------------
bool_t UartPort::write(const uint8_t* data, uint_t size, const ConnectionMeta& meta)
{
    bool_t written = false;

    if (!m_lock)
    {
        if (data && size)
        {
            uint_t bufSize = size;

            if (m_codec)
            {
                bufSize += (size / 254) + 8;
            }

            TxRxBuf* buf = (TxRxBuf*)m_tx.newItem(sizeof(TxRxBuf) + bufSize);
            if (buf)
            {
                buf->data = reinterpret_cast<uint8_t*>(buf) + sizeof(TxRxBuf);
                if (m_codec)
                {
                    size = m_codec->encode(data, size, buf->data, bufSize);
                    m_tx.reduceSize(sizeof(TxRxBuf) + size);
                }
                else
                {
				    Mem::memcpy(buf->data, data, size);
                }
                buf->size = size;
                buf->baudrate = meta.baudrate;
                m_tx.push();
                written = true;
                m_serialPort.write(buf->data, buf->size, meta.baudrate);
            }
			else
			{
				debugLog("UartPort", "%s tx buffer full. Discarding packet", name.c_str());
				return false;
			}
        }
        else
        {
            written = m_serialPort.write(data, size, meta.baudrate);
        }
    }

    return written;
}
//--------------------------------------------------------------------------------------------------
void UartPort::discoverIslDevices(uint16_t pid, uint16_t pn, uint16_t sn)
{
    for (uint_t i = 0; i < defaultBaudrates.size(); i++)
    {
        SysPort::discoverIslDevices(pid, pn, sn, ConnectionMeta(defaultBaudrates[i]), discoveryTimeoutMs, 1);
    }
}
//--------------------------------------------------------------------------------------------------
void UartPort::discoverIslDevices(uint16_t pid, uint16_t pn, uint16_t sn, uint32_t baudrate, uint_t timeoutMs)
{
    SysPort::discoverIslDevices(pid, pn, sn, ConnectionMeta(baudrate), timeoutMs, 1);
}
//--------------------------------------------------------------------------------------------------
void UartPort::discoverNmeaDevices()
{
    for (uint_t i = 0; i < defaultBaudrates.size(); i++)
    {
        nemaDiscovery(ConnectionMeta(defaultBaudrates[i]), 1500);
    }
}
//--------------------------------------------------------------------------------------------------
void UartPort::discoverNmeaDevices(uint32_t baudrate, uint_t timeoutMs)
{
    nemaDiscovery(ConnectionMeta(baudrate), timeoutMs);
}
//--------------------------------------------------------------------------------------------------
bool_t UartPort::process()
{
    TxRxBuf* buf;
    uint_t bytesToProcess;
    uint_t frameSize;
    
    if (m_eventOpen)
    {
        m_eventOpen = false;
        if (m_threadOpen.joinable())
        {
            m_threadOpen.join();
        }
        rxDataCallback(nullptr, 0, 0);
        debugLog("SysPort", "%s opening", name.c_str());
        onOpen(*this, m_isOpen);
    }
    
    if (m_eventClose)
    {
        if (m_threadClose.joinable())
        {
            m_threadClose.join();
        }
        m_eventClose = false;
        m_rx.reset();
        SysPort::close();
    }

    while (buf = reinterpret_cast<TxRxBuf*>(m_rx.peekNextItem()))
    {
        m_rxBytesCount += buf->size;
        ConstBuffer buffer = { buf->data, buf->size };
        onRxData(*this, buffer);

        if (m_codec)
        {
            bytesToProcess = buf->size;

            while (bytesToProcess && m_codec)
            {
                frameSize = m_codec->decode(&buf->data[buf->size - bytesToProcess], &bytesToProcess);

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
void UartPort::rxDataCallback(const uint8_t* data, uint_t size, uint32_t baudrate)
{
    if (m_rxBuf)
    {
        if (m_rxBuf->size > size)
        {
            m_rxBuf->size = size;
            m_rx.reduceSize(sizeof(TxRxBuf) + size);
        }
        m_rxBuf->baudrate = baudrate;
        m_rx.push();
        m_rxBuf = nullptr;
        // sdk event
    }

    TxRxBuf* buf = reinterpret_cast<TxRxBuf*>(m_rx.newItem(sizeof(TxRxBuf) + m_rxBufSize));
    if (buf)
    {
        buf->data = (uint8_t*)buf + sizeof(TxRxBuf);
        buf->size = m_rxBufSize;
        buf->baudrate = 0;
        m_rxBuf = buf;
        m_serialPort.setRxBuffer(buf->data, buf->size);
    }
}
//--------------------------------------------------------------------------------------------------
void UartPort::txCompeteCallback(const uint8_t* data, uint_t bytesWritten)
{
    txComplete(data, bytesWritten);
    m_tx.pop();
    TxRxBuf* buf = reinterpret_cast<TxRxBuf*>(m_tx.peekNextItem());
    if (buf)
    {
        m_serialPort.write(buf->data, buf->size, buf->baudrate);
    }
}
//--------------------------------------------------------------------------------------------------
void UartPort::uartEvent(Uart::Events event)
{
    if (event == Uart::Events::Error)
	{
        m_portError = true;
	}
    else if (event == Uart::Events::Open)
	{
        m_isOpen = m_serialPort.isOpen();
		m_eventOpen = true;
	}
	else if (event == Uart::Events::Close)
	{
        m_eventClose = true;
	}
}
//--------------------------------------------------------------------------------------------------
