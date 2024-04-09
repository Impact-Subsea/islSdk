//------------------------------------------ Includes ----------------------------------------------

#include "uartPort.h"
#include "platform/debug.h"

using namespace IslSdk;

std::vector<uint32_t> UartPort::defaultBaudrates = { 115200, 9600, 57600, 38400, 19200, 4800 };

//--------------------------------------------------------------------------------------------------
UartPort::UartPort(const std::string& name) : SysPort(name, Type::Serial, 250)
{
    m_rxBuf = nullptr;
    m_uart.rxDataEvent.connect(m_slotRxData);
    m_uart.txCompleteEvent.connect(m_slotTxCompete);
    m_uart.errorEvent.connect(this, &UartPort::errorCallback);
}
//--------------------------------------------------------------------------------------------------
UartPort::~UartPort()
{
    close();
}
//--------------------------------------------------------------------------------------------------
bool_t UartPort::open()
{
    if (!m_isOpen)
    {
        m_rx.reset();
        m_tx.reset();
        m_rxBuf = nullptr;
        m_isOpen = m_uart.open();
        rxDataCallback(nullptr, 0, 0);
        debugLog("SysPort", "%s opening", name.c_str());
        onOpen(*this, !m_isOpen);
    }
    return m_isOpen;
}
//--------------------------------------------------------------------------------------------------
void UartPort::close()
{
    if (m_isOpen)
    {
        m_uart.close();
        m_rx.reset();
        SysPort::close();
    }
}
//--------------------------------------------------------------------------------------------------
bool_t UartPort::write(const uint8_t* data, uint_t size, const ConnectionMeta& meta)
{
    bool_t written = false;

    if (!m_lock)
    {
        if (data && size)
        {
            uint_t maxEncodedSize = size + (size / 254) + 8;
            TxRxBuf* buf = (TxRxBuf*)m_tx.newItem(sizeof(TxRxBuf) + maxEncodedSize);
            if (buf)
            {
                buf->data = reinterpret_cast<uint8_t*>(buf) + sizeof(TxRxBuf);
                size = m_codec->encode(data, size, buf->data, maxEncodedSize);
                buf->size = size;
                buf->baudrate = meta.baudrate;
                m_tx.reduceSize(sizeof(TxRxBuf) + size);
                m_tx.push();
                data = buf->data;
                written = true;
                m_uart.write(data, size, meta.baudrate);
            }
            else
            {
                debugLog("UartPort", "%s tx buffer full. Discarding packet", name.c_str());
                return false;
            }
        }
        else
        {
            written = m_uart.write(data, size, meta.baudrate);
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

    while (buf = reinterpret_cast<TxRxBuf*>(m_rx.peekNextItem()))
    {
        m_rxBytesCount += buf->size;
        ConstBuffer buffer = { buf->data, buf->size };
        onRxData(*this, buffer);

        if (m_codec)
        {
            bytesToProcess = buf->size;

            while (bytesToProcess)
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
        m_uart.setRxBuffer(buf->data, buf->size);
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
        m_uart.write(buf->data, buf->size, buf->baudrate);
    }
}
//--------------------------------------------------------------------------------------------------
void UartPort::errorCallback()
{
    m_portError = true;
}
//--------------------------------------------------------------------------------------------------
