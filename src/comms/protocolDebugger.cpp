//------------------------------------------ Includes ----------------------------------------------

#include "protocolDebugger.h"
#include "comms/islHdlc.h"
#include "platform/timeUtils.h"
#include <sstream>
#include <iostream>
#include <iomanip>

using namespace IslSdk;

//--------------------------------------------------------------------------------------------------
ProtocolDebugger::ProtocolDebugger(const std::string& name) : m_name(name)
{
    m_txProcessedBytes = 0;
    m_rxProcessedBytes = 0;
    m_showPayload = false;
    m_startTimeMs = Time::getTimeMs();
}
//--------------------------------------------------------------------------------------------------
ProtocolDebugger::~ProtocolDebugger()
{
}
//--------------------------------------------------------------------------------------------------
void ProtocolDebugger::monitorPort(const SysPort::SharedPtr& port)
{
    if (m_port)
    {
        m_port->onRxData.disconnect(slotOnRxData);
        m_port->onTxData.disconnect(slotOnTxData);
    }
    m_startTimeMs = Time::getTimeMs();
    m_port = port;
    m_port->onRxData.connect(slotOnRxData);
    m_port->onTxData.connect(slotOnTxData);
}
//--------------------------------------------------------------------------------------------------
void ProtocolDebugger::rxData(SysPort& port, const uint8_t* data, uint_t size)
{
    if (port.type == SysPort::Type::Serial || port.type == SysPort::Type::Sol)
    {
        uint_t bytesToProcess = size;

        while (bytesToProcess)
        {
            uint_t bytesProcessed = size - bytesToProcess;
            uint_t frameSize = m_rxCodec.decode(&data[size - bytesToProcess], &bytesToProcess);
            m_rxProcessedBytes += (size - bytesToProcess) - bytesProcessed;

            if (frameSize)
            {
                decodeFrame(&m_rxCodec.m_frameBuf[0], frameSize, m_rxProcessedBytes, false);
                m_rxProcessedBytes = 0;
            }
        }
    }
    else
    {
        decodeFrame(data, size, size, false);
    }
}
//--------------------------------------------------------------------------------------------------
void ProtocolDebugger::txData(SysPort& port, const uint8_t* data, uint_t size)
{
    if (port.type == SysPort::Type::Serial || port.type == SysPort::Type::Sol)
    {
        uint_t bytesToProcess = size;

        while (bytesToProcess)
        {
            uint_t bytesProcessed = size - bytesToProcess;
            uint_t frameSize = m_txCodec.decode(&data[size - bytesToProcess], &bytesToProcess);
            m_txProcessedBytes += (size - bytesToProcess) - bytesProcessed;

            if (frameSize)
            {
                decodeFrame(&m_txCodec.m_frameBuf[0], frameSize, m_txProcessedBytes, true);
                m_txProcessedBytes = 0;
            }
        }
    }
    else
    {
        decodeFrame(data, size, size, true);
    }
}
//--------------------------------------------------------------------------------------------------
void ProtocolDebugger::decodeFrame(const uint8_t* frame, uint_t size, uint_t totalByteCount, bool_t isTx)
{
    const struct
    {
        const char* black = "\033[30m";
        const char* red = "\033[31m";
        const char* green = "\033[32m";
        const char* yellow = "\033[33m";
        const char* blue = "\033[34m";
        const char* magenta = "\033[35m";
        const char* cyan = "\033[36m";
        const char* white = "\033[37m";
        const char* grey = "\033[90m";
        const char* reset = "\033[39m";
    } Text;

    uint64_t timeMs = Time::getTimeMs() - m_startTimeMs;
    std::ostringstream msg;
    IslHdlcPacket packet;
    const std::string txRx(isTx ? " TX" : " RX");

    if (packet.fromFrame(frame, size))
    {
        const char* frameTypes[] = { "\033[36mI frame", "\033[33mS frame", "\033[36mI frame", "\033[33mU frame" };
        const char* dirCol[] = { "\033[32m", "\033[31m" };
        const char* dir[] = { "<- ", "-> " };
        const char* pf[] = { "\033[34m---- ", "\033[34m---- ", "\033[33mfinal", "\033[33mpoll " };
        const char* uCode[] = { "discover", "connect", "disconnect" };
        const char* sCode[] = { "RR", "RNR", "REJ", "SREJ" };
        uint_t mode = (static_cast<uint_t>(packet.header.pf) << 1) | static_cast<uint_t>(packet.header.cr);
        msg << m_name << dirCol[isTx] << txRx << Text.white << " " << timeMs << "ms\t" << Text.magenta << "PC " << dirCol[packet.header.cr] << dir[packet.header.cr] << pf[mode] << dirCol[packet.header.cr] << dir[packet.header.cr] << Text.magenta << "Device" << Text.cyan << "["<< static_cast<uint_t>(packet.header.address) << "] " << frameTypes[static_cast<uint_t>(packet.header.type) >> 3] << Text.magenta;
        if (packet.header.type == IslHdlcPacket::FrameType::U)
        {
            msg << " cmd:" << uCode[static_cast<uint_t>(packet.header.uCode)];
        }
        else if (packet.header.type == IslHdlcPacket::FrameType::S)
        {
            msg << " cmd:" << sCode[static_cast<uint_t>(packet.header.sCode)] << dirCol[packet.header.cr] << " ack:" << static_cast<uint_t>(packet.header.ack);
        }
        else
        {
            msg << dirCol[packet.header.cr] << " seq:" << static_cast<uint_t>(packet.header.seq) << " ack:" << static_cast<uint_t>(packet.header.ack);
        }
        msg << Text.grey << "\tPacket size:" << size << " bytes (" << totalByteCount << " on the wire)\tPayload: " << packet.size << "\n";

        if (packet.size && m_showPayload)
        {
            for (size_t i = 0; i < packet.size; i++)
            {
                msg << std::setfill('0') << std::setw(2) << std::hex << (uint_t)packet.payload[i] << " ";
                if ((i + 1) % 31 == 0)
                {
                    msg << "\n";
                }
            }
            msg << "\n";
        }
    }
    else
    {
        msg << "Frame was corrupt. " << totalByteCount << " bytes\n";
    }
    msg << Text.white;
    std::cout << msg.str();
}
//--------------------------------------------------------------------------------------------------
