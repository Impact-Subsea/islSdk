//------------------------------------------ Includes ----------------------------------------------

#include "islHdlc.h"
#include "utils/crc.h"
#include "platform/mem.h"
#include "maths/maths.h"
#include "platform/timeUtils.h"
#include "platform/debug.h"

using namespace IslSdk;

#define LEGACY_DISCOVERY_SUPPORT

uint8_t IslHdlc::m_addressCounter = 0;

struct TxItem
{
    bool_t send;
    uint8_t sendCount;
    uint8_t seq;
    uint_t size;
    uint8_t* frame;
};

struct HeaderMasks
{
    static const uint8_t version = 0x07 << 0;
    static const uint8_t frameType = 0x03 << 3;
    static const uint8_t pf = 1 << 5;
    static const uint8_t cr = 1 << 6;
};

//--------------------------------------------------------------------------------------------------
bool_t IslHdlcPacket::fromFrame(const uint8_t* frame, uint_t frameSize)
{
    uint16_t frameCrc;
    uint16_t crc;

    if (frameSize >= overHeadSize)
    {
        frameCrc = Mem::get16Bit(&frame[frameSize - 2]);
        crc = crc16(0xffff, frame, frameSize - 2);

        if (frameCrc == crc)
        {
            if ((frame[0] & HeaderMasks::version) == 2)
            {
                header.protocolVersion = *frame & HeaderMasks::version;
                header.type = static_cast<FrameType>((*frame & HeaderMasks::frameType));
                header.pf = (*frame & HeaderMasks::pf) != 0;
                header.cr = (*frame & HeaderMasks::cr) != 0;
                frame++;
                header.ack = *frame++;
                header.uCode = static_cast<UframeCode>(0);
                header.seq = *frame++;
                header.address = *frame++;
                size = frameSize - overHeadSize;
                payload = frame;
                return true;
            }
#ifdef LEGACY_DISCOVERY_SUPPORT
            else if ((frame[0] & HeaderMasks::version) == 1 && (frame[1] == 0x82))
            {
                header.protocolVersion = 2;
                header.type = FrameType::U;
                header.pf = true;
                header.cr = false;
                header.ack = 0;
                header.uCode = UframeCode::Discover;
                header.address = 0;
                size = 14;
                frame += 10;
                static uint8_t buf[14];
                buf[0] = frame[4];
                buf[1] = frame[5];
                buf[2] = frame[0];
                buf[3] = frame[1];
                buf[4] = frame[2];
                buf[5] = frame[3];
                buf[6] = frame[6];
                buf[7] = frame[7];
                buf[8] = frame[8];
                buf[9] = frame[9];
                buf[10] = frame[10];
                buf[11] = frame[11];
                buf[12] = frame[12];
                buf[13] = frame[13];
                payload = &buf[0];
                return true;
            }
#endif
        }
        else
        {
            Debug::log(Debug::Severity::Warning, "IslHdlc", "Frame CRC error calculated 0x%04x should be 0x%04x", FMT_X(crc), FMT_X(frameCrc));
        }
    }
    return false;
}
//--------------------------------------------------------------------------------------------------
const std::vector<uint8_t> IslHdlc::BuildDiscovery(uint16_t pid, uint16_t pn, uint16_t sn)
{
#ifdef LEGACY_DISCOVERY_SUPPORT
    std::vector<uint8_t> buf(12 + ((pid != 0xffff) * 2));
    uint8_t* ptr = &buf[0];

    *ptr++ = 1;
    *ptr++ = 0x42;
    Mem::pack16Bit(&ptr, pn);
    Mem::pack16Bit(&ptr, sn);
    Mem::pack32Bit(&ptr, 1);

    if (pid != 0xffff)
    {
        Mem::pack16Bit(&ptr, pid);
    }

    uint16_t crc = crc16(0xffff, &buf[0], ptr - &buf[0]);
    Mem::pack16Bit(&ptr, crc);
    return buf;
#else
    std::vector<uint8_t> buf(12);
    uint8_t* ptr = &buf[0];

    *ptr++ = HeaderMasks::cr | HeaderMasks::pf | (static_cast<uint8_t>(IslHdlcPacket::FrameType::U) & HeaderMasks::frameType) | (2 & HeaderMasks::version); // Set CR bit, set PF bit, U frame, version 2
    *ptr++ = 0;
    *ptr++ = static_cast<uint8_t>(IslHdlcPacket::UframeCode::Discover);
    *ptr++ = 0xff;
    Mem::pack16Bit(&ptr, pid);
    Mem::pack16Bit(&ptr, pn);
    Mem::pack16Bit(&ptr, sn);
    uint16_t crc = crc16(0xffff, &buf[0], 10);
    Mem::pack16Bit(&ptr, crc);
    return buf;
#endif
}
//--------------------------------------------------------------------------------------------------
IslHdlc::IslHdlc(uint_t mtu, uint_t txQueSize, uint8_t armWindowSize, uint8_t nrmWindowSize) :
    m_mtu(mtu),
    m_txPacketQ(txQueSize),
    m_armWindowSize(armWindowSize),
    m_nrmWindowSize(nrmWindowSize),
    m_timeoutMs(500),
    id(static_cast<uint32_t>(Math::randomNum(1, Math::maxUint32)))
{
    if (m_addressCounter == 0)
    {
        m_addressCounter = static_cast<uint8_t>(Math::randomNum(1, 254));
    }

    if (m_addressCounter == 0 || m_addressCounter == 255)
    {
        m_addressCounter = 1;
    }

    m_address = m_addressCounter++;
    m_windowSize = nrmWindowSize;
    reset();
}
//--------------------------------------------------------------------------------------------------
IslHdlc::~IslHdlc()
{
}
//--------------------------------------------------------------------------------------------------
void IslHdlc::reset()
{
    if (m_connection)
    {
        m_connection->sysPort->unblock(id);
    }

    m_txSeq = 0;
    m_nextRxSeq = 0;
    m_windowLevel = 0;
    m_txMsgCount = 0;
    m_pendingTxCount = 0;
    m_iFramesSinceAckSent = 0;
    m_checkpointTxSeq = 0;
    m_lastTxSeq = 0xff;
    m_connected = false;
    m_pollFlag = false;
    m_finalFlag = false;
    m_waitForFinalFlag = false;
    m_talkToken = true;
    m_isNrm = true;
    m_blockRej = false;
    m_timeoutCount = 0;
    m_timeout = 0;
    m_pollFlagTxTime = 0;
    m_txPacketQ.reset();
    m_ctrlFrameSize = 0;
    m_multiFrameBuffering = false;
    m_multiFrameBuf.clear();
    m_packetCount.tx = 0;
    m_packetCount.rx = 0;
    m_packetCount.resent = 0;
    m_packetCount.rxMissed = 0;
}
//--------------------------------------------------------------------------------------------------
void IslHdlc::setCommsTimeout(uint32_t timeoutMs)
{
    m_timeoutMs = timeoutMs;
}
//--------------------------------------------------------------------------------------------------
void IslHdlc::connect(uint16_t pn, uint16_t sn, uint32_t timeout)
{
    uint8_t buf[10];
    uint8_t* ptr;

    ptr = &buf[0];
    Mem::pack16Bit(&ptr, pn);
    Mem::pack16Bit(&ptr, sn);
    Mem::pack32Bit(&ptr, timeout);
    *ptr++ = m_address;
    *ptr = 0;

    debugLog("IslHdlc", "Queuing connection request to %04u.%04u, assigning address:%u", FMT_U(pn), FMT_U(sn), FMT_U(m_address));
    sendUFrame(IslHdlcPacket::UframeCode::Connect, 0xff, &buf[0], sizeof(buf));
}
//--------------------------------------------------------------------------------------------------
void IslHdlc::disconnect()
{
    sendUFrame(IslHdlcPacket::UframeCode::Disconnect, m_address, nullptr, 0);
    process();
    if (m_connected)
    {
        m_connected = false;
        hdlcConnectionEvent(false);
    }
    reset();
}
//--------------------------------------------------------------------------------------------------
void IslHdlc::setNrmMode(bool_t isNrm)
{
    m_isNrm = isNrm;
    if (m_isNrm && m_armWindowSize > m_nrmWindowSize)
    {
        m_windowSize = m_nrmWindowSize;
    }
    else
    {
        m_windowSize = m_armWindowSize;
    }
}
//--------------------------------------------------------------------------------------------------
bool_t IslHdlc::process()
{
    bool_t didSend = false;
    uint_t windowSize;

    if (m_connected)
    {
        uint64_t time = Time::getTimeMs();
        if (m_waitForFinalFlag)
        {
            if (time >= m_timeout || time >= (m_pollFlagTxTime + m_timeoutMs + 4500))
            {
                if (m_connection)
                {
                    m_connection->sysPort->unblock(id);
                }
                m_waitForFinalFlag = false;
                m_talkToken = true;
                debugLog("IslHdlc", "Timeout, no final flag in responce to poll");
                if (!timeoutEvent())
                {
                    if (m_connected)
                    {
                        m_connected = false;
                        hdlcConnectionEvent(false);
                    }
                    debugLog("IslHdlc", "Terminating connection");
                    reset();
                }
                else
                {
                    m_timeoutCount++;
                }
            }
        }
        else if (m_isNrm || time >= (m_pollFlagTxTime + 250))
        {
            if (!m_isNrm || (m_isNrm && m_pendingTxCount == 0))
            {
                m_pollFlag = true;
            }

            if (m_pendingTxCount == 0)
            {
                sendSFrame(IslHdlcPacket::SframeCode::Rr);
            }
        }
    }

    if (m_talkToken && m_ctrlFrameSize)
    {
        didSend = transmitFrame(&m_ctrlFrameBuf[0], m_ctrlFrameSize);
        if (!didSend)
        {
            return false;
        }
        m_ctrlFrameSize = 0;
    }


    windowSize = m_windowSize;
    TxItem* msg = reinterpret_cast<TxItem*>(m_txPacketQ.peekNextItem());

    while (m_talkToken && msg && windowSize)
    {
        if (msg->send)
        {
            if (m_isNrm && m_pendingTxCount == 1)
            {
                m_pollFlag = true;
            }

            didSend = transmitFrame(&msg->frame[0], msg->size);
            if (!didSend)
            {
                return false;
            }

            msg->send = false;
            m_pendingTxCount--;
            if (msg->sendCount == 0)
            {
                m_windowLevel++;
            }
            else
            {
                m_packetCount.resent++;
            }
            msg->sendCount++;
        }
        msg = reinterpret_cast<TxItem*>(m_txPacketQ.peekNextItem(msg));
        windowSize--;
    }

    return didSend;
}
//--------------------------------------------------------------------------------------------------
void IslHdlc::processPacket(const IslHdlcPacket& packet)
{
    m_packetCount.rx++;
    if (packet.header.pf && !packet.header.cr)
    {
        m_talkToken = true;
    }

    m_finalFlag = m_finalFlag || (packet.header.pf && packet.header.cr);
    m_waitForFinalFlag = m_waitForFinalFlag && !(packet.header.pf && !packet.header.cr);
    m_timeoutCount = 0;

    m_timeout = Time::getTimeMs() + m_timeoutMs;

    if (m_connection->sysPort->type == SysPort::Type::Serial || m_connection->sysPort->type == SysPort::Type::Sol)
    {
        m_timeout += static_cast<uint64_t>(static_cast<real_t>(m_mtu) * (static_cast<real_t>(10000.0) / static_cast<real_t>(m_connection->meta.baudrate)));
    }

    if (m_talkToken)
    {
        if (m_connection)
        {
            m_connection->sysPort->unblock(id);
        }
    }

    if (packet.header.type == IslHdlcPacket::FrameType::U)
    {
        processUFrame(packet);
    }
    else if (m_connected)
    {
        processAck(packet.header);

        if (packet.header.type == IslHdlcPacket::FrameType::S)
        {
            processSFrame(packet);
        }
        else
        {
            processIFrame(packet);

            m_iFramesSinceAckSent++;

            if (m_pendingTxCount == 0 && m_iFramesSinceAckSent >= 8)
            {
                sendSFrame(IslHdlcPacket::SframeCode::Rr);
                if ((m_pendingTxCount == 0 && m_iFramesSinceAckSent >= 8) || (m_isNrm && m_talkToken))
                {
                    sendSFrame(IslHdlcPacket::SframeCode::Rr);
                    if (transmitFrame(&m_ctrlFrameBuf[0], m_ctrlFrameSize))
                    {
                        m_ctrlFrameSize = 0;
                    }
                }
            }
        }
    }
}

//--------------------------------------------------------------------------------------------------
void IslHdlc::send(const uint8_t* data, uint_t size)
{
    if (m_connected)
    {
        while (size)
        {
            uint_t txSize = Math::min<uint_t>(m_mtu, size);
            TxItem* item = reinterpret_cast<TxItem*>(m_txPacketQ.newItem(sizeof(TxItem) + txSize + IslHdlcPacket::overHeadSize));

            if (item)
            {
                item->send = true;
                item->sendCount = 0;
                item->seq = m_txSeq;
                item->size = txSize + IslHdlcPacket::overHeadSize;
                item->frame = reinterpret_cast<uint8_t*>(item) + sizeof(TxItem);
                item->frame[0] = txSize == size ? static_cast<uint8_t>(IslHdlcPacket::FrameType::I) : static_cast<uint8_t>(IslHdlcPacket::FrameType::Im);
                item->frame[1] = 0;
                item->frame[2] = m_txSeq;
                item->frame[3] = m_address;
                Mem::memcpy(&item->frame[4], data, txSize);
                m_txPacketQ.push();
                m_txSeq++;
                m_txMsgCount++;
                m_pendingTxCount += m_txMsgCount <= m_windowSize;
            }
            else
            {
                debugLog("IslHdlc", "Tx window full, discarding data");
                break;
            }

            data += txSize;
            size -= txSize;
        }
    }
}
//--------------------------------------------------------------------------------------------------
void IslHdlc::sendSFrame(IslHdlcPacket::SframeCode code)
{
    if (m_ctrlFrameSize == 0)
    {
        m_ctrlFrameSize = IslHdlcPacket::overHeadSize;
        m_ctrlFrameBuf[0] = static_cast<uint8_t>(IslHdlcPacket::FrameType::S);
        m_ctrlFrameBuf[1] = 0;
        m_ctrlFrameBuf[2] = static_cast<uint8_t>(code);
        m_ctrlFrameBuf[3] = m_address;
    }
}
//--------------------------------------------------------------------------------------------------
void IslHdlc::sendUFrame(IslHdlcPacket::UframeCode code, uint8_t address, void* data, uint_t size)
{
    if (m_ctrlFrameSize == 0)
    {
        m_ctrlFrameSize = IslHdlcPacket::overHeadSize + size;
        m_ctrlFrameBuf[0] = static_cast<uint8_t>(IslHdlcPacket::FrameType::U);
        m_ctrlFrameBuf[1] = 0;
        m_ctrlFrameBuf[2] = static_cast<uint8_t>(code);
        m_ctrlFrameBuf[3] = address;
        Mem::memcpy(&m_ctrlFrameBuf[4], data, size);
    }
}
//--------------------------------------------------------------------------------------------------
void IslHdlc::processIFrame(const IslHdlcPacket& packet)
{
    if (packet.header.seq == m_nextRxSeq)
    {
        m_blockRej = false;
        m_nextRxSeq++;

        if (packet.header.type == IslHdlcPacket::FrameType::Im && !m_multiFrameBuffering)
        {
            m_multiFrameBuf.clear();
            m_multiFrameBuffering = true;
        }

        if (m_multiFrameBuffering)
        {
            if (m_multiFrameBuf.capacity() < m_multiFrameBuf.size() + packet.size)
            {
                m_multiFrameBuf.reserve((m_multiFrameBuf.size() + packet.size) * 2);
            }

            m_multiFrameBuf.insert(m_multiFrameBuf.end(), &packet.payload[0], &packet.payload[packet.size]);

            if (packet.header.type == IslHdlcPacket::FrameType::I)
            {
                newPacketEvent(&m_multiFrameBuf[0], m_multiFrameBuf.size());
                m_multiFrameBuf.clear();
                m_multiFrameBuffering = false;
            }
        }
        else
        {
            newPacketEvent(packet.payload, packet.size);
        }
    }
    else if (!m_blockRej)
    {
        debugLog("IslHdlc", "Rx sequence error, received:%u expected:%u. Sending REJ", FMT_U(packet.header.seq), FMT_U(m_nextRxSeq));
        m_blockRej = true;
        m_packetCount.rxMissed += (packet.header.seq - m_nextRxSeq) & 0xff;
        sendSFrame(IslHdlcPacket::SframeCode::Rej);
    }
}
//--------------------------------------------------------------------------------------------------
void IslHdlc::processSFrame(const IslHdlcPacket& packet)
{
    TxItem* msg;

    switch (packet.header.sCode)
    {
    case IslHdlcPacket::SframeCode::Rr:
        break;

    case IslHdlcPacket::SframeCode::Rnr:
        // stop I frames
        break;

    case IslHdlcPacket::SframeCode::Rej:
        debugLog("IslHdlc", "Received REJ, last ack sequence: %u", FMT_U(packet.header.ack));
        msg = reinterpret_cast<TxItem*>(m_txPacketQ.peekNextItem());
        if (msg != nullptr && msg->seq == packet.header.ack)
        {
            while (msg != nullptr && !msg->send)
            {
                msg->send = true;
                msg = reinterpret_cast<TxItem*>(m_txPacketQ.peekNextItem(msg));
                m_pendingTxCount++;
            }
        }
        break;

    case IslHdlcPacket::SframeCode::Srej:
        msg = reinterpret_cast<TxItem*>(m_txPacketQ.peekNextItem());
        if (msg != nullptr && !msg->send && msg->seq == packet.header.ack)
        {
            msg->send = true;
            m_pendingTxCount++;
        }
        break;
    }
}
//--------------------------------------------------------------------------------------------------
void IslHdlc::processUFrame(const IslHdlcPacket& packet)
{
    bool_t halfDuplex;

    switch (packet.header.uCode)
    {
    case IslHdlcPacket::UframeCode::Discover:
        break;

    case IslHdlcPacket::UframeCode::Connect:
        halfDuplex = packet.payload[0] != 0;
        debugLog("IslHdlc", "%s connection accepted from device with address:%u", halfDuplex ? "Half duplex" : "Full duplex", FMT_U(packet.header.address));
        reset();
        setNrmMode(halfDuplex);
        m_connected = true;
        hdlcConnectionEvent(true);
        break;

    case IslHdlcPacket::UframeCode::Disconnect:
        if (m_connected)
        {
            m_connected = false;
            hdlcConnectionEvent(false);
        }
        reset();
        break;
    }
}
//--------------------------------------------------------------------------------------------------
void IslHdlc::processAck(const IslHdlcPacket::Header& hdr)
{
    TxItem* msg;
    uint_t windowCount;

    msg = reinterpret_cast<TxItem*>(m_txPacketQ.peekNextItem());

    if (msg != nullptr)
    {
        windowCount = m_windowLevel;
        while (msg != nullptr && msg->seq != hdr.ack && windowCount)
        {
            if (m_checkpointTxSeq == msg->seq)
            {
                m_checkpointTxSeq++;
            }
            if (msg->send)
            {
                m_pendingTxCount--;
            }
            m_txPacketQ.pop();
            msg = reinterpret_cast<TxItem*>(m_txPacketQ.peekNextItem());
            m_windowLevel--;
            m_txMsgCount--;
            windowCount--;
            m_pendingTxCount += m_txMsgCount >= m_windowSize;
        }

        if (hdr.pf && msg != nullptr && (m_checkpointTxSeq != hdr.ack))
        {
            windowCount = m_windowSize;
            while (msg != nullptr && windowCount)
            {
                if (!msg->send)
                {
                    m_pendingTxCount++;
                    msg->send = true;
                }
                windowCount--;
                msg = reinterpret_cast<TxItem*>(m_txPacketQ.peekNextItem(msg));
            }
        }
    }
}
//--------------------------------------------------------------------------------------------------
bool_t IslHdlc::transmitFrame(uint8_t* frame, uint_t size)
{
    bool_t written = false;

    if (m_connection)
    {
        if (!m_connection->sysPort->isBlocked())
        {
            IslHdlcPacket::FrameType frameType = static_cast<IslHdlcPacket::FrameType>(frame[0] & HeaderMasks::frameType);

            if (frameType == IslHdlcPacket::FrameType::U)
            {
                m_pollFlag = true;
            }
            else
            {
                frame[1] = m_nextRxSeq;
            }

            frame[0] = HeaderMasks::cr | (((m_pollFlag || m_finalFlag) << 5) & HeaderMasks::pf) | (static_cast<uint8_t>(frameType) & HeaderMasks::frameType) | 2 & HeaderMasks::version;

            uint16_t crc = crc16(0xffff, frame, size - 2);
            Mem::pack16Bit(&frame[size - 2], crc);

            written = m_connection->sysPort->write(frame, size, m_connection->meta);

            if (written)
            {
                if (frameType != IslHdlcPacket::FrameType::U)
                {
                    m_iFramesSinceAckSent = 0;
                    if (frameType != IslHdlcPacket::FrameType::S)
                    {
                        m_lastTxSeq = frame[2];
                    }

                    if (m_pollFlag)
                    {
                        m_checkpointTxSeq = m_lastTxSeq + 1;
                    }
                }

                if (m_pollFlag)
                {
                    if (m_isNrm)
                    {
                        m_connection->sysPort->block(id);
                    }
                    m_pollFlagTxTime = Time::getTimeMs();
                    m_timeout = m_pollFlagTxTime + (m_timeoutMs * (static_cast<uint_t>(1) << m_timeoutCount));
                    m_waitForFinalFlag = true;
                    if (m_connection->sysPort->type == SysPort::Type::Serial || m_connection->sysPort->type == SysPort::Type::Sol)
                    {
                        m_timeout += static_cast<uint64_t>(static_cast<real_t>(size + m_mtu) * (static_cast<real_t>(10000.0) / static_cast<real_t>(m_connection->meta.baudrate)));
                    }
                }

                m_talkToken = !m_isNrm || (m_talkToken && !m_pollFlag);
                m_finalFlag = m_finalFlag && m_pollFlag;
                m_pollFlag = false;
                m_packetCount.tx++;
            }
        }
    }
    return written;
}
//--------------------------------------------------------------------------------------------------
