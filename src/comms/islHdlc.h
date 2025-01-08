#ifndef ISLHDLC_H_
#define ISLHDLC_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"
#include "comms/ports/sysPort.h"
#include "types/queue.h"
#include <vector>
#include <memory>

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    class IslHdlcPacket
    {
    public:
        enum class FrameType : uint8_t { I = (0 << 3), S = (1 << 3), Im = (2 << 3), U = (3 << 3) };
        enum class SframeCode : uint8_t { Rr, Rnr, Rej, Srej };
        enum class UframeCode : uint8_t { Discover, Connect, Disconnect };
        static const uint_t overHeadSize = 6;

        struct Header
        {
            uint8_t protocolVersion;
            FrameType type;
            bool_t pf;
            bool_t cr;
            uint8_t ack;
            union
            {
                uint8_t seq;
                SframeCode sCode;
                UframeCode uCode;
            };
            uint8_t address;
        } header;

        const uint8_t* payload;
        uint_t size;

        bool_t fromFrame(const uint8_t* frame, uint_t size);
    };

    struct Connection
    {
        const std::shared_ptr<SysPort> sysPort;
        ConnectionMeta meta;
        Connection(const std::shared_ptr<SysPort>& sysPort, const ConnectionMeta& meta) : sysPort(sysPort), meta(meta) {}
    };

    class IslHdlc
    {
    public:
        const uint32_t id;
        IslHdlc(uint_t mtu = 1050, uint_t txQueSize = 1024 * 8, uint8_t armWindowSize = 15, uint8_t nrmWindowSize = 8);
        virtual ~IslHdlc();
        void setCommsTimeout(uint32_t timeoutMs);
        void processPacket(const IslHdlcPacket& packet);
        static const std::vector<uint8_t> BuildDiscovery(uint16_t pid, uint16_t pn, uint16_t sn);
        void disconnect();

    protected:
        std::unique_ptr<Connection> m_connection;
        uint8_t m_address;
        bool_t m_connected;
        uint_t m_timeoutCount;
        bool_t m_isNrm;
        uint_t m_timeoutMs;
        struct
        {
            uint_t tx;
            uint_t rx;
            uint_t resent;
            uint_t rxMissed;
        } m_packetCount;
        virtual void hdlcConnectionEvent(bool_t connected) = 0;
        virtual bool_t timeoutEvent() = 0;
        virtual void newPacketEvent(const uint8_t* data, uint_t size) = 0;

        void connect(uint16_t pn, uint16_t sn, uint32_t timeout);
        void setNrmMode(bool_t isNrm);
        bool_t process();
        void send(const uint8_t* data, uint_t size);

    private:
        static uint8_t m_addressCounter;
        const uint8_t m_nrmWindowSize;
        const uint8_t m_armWindowSize;
        const uint_t m_mtu;

        Queue m_txPacketQ;
        uint8_t m_txSeq;
        uint8_t m_nextRxSeq;
        uint8_t m_windowLevel;
        uint8_t m_windowSize;
        uint8_t m_checkpointTxSeq;
        uint8_t m_lastTxSeq;
        bool_t m_pollFlag;
        bool_t m_finalFlag;
        bool_t m_waitForFinalFlag;
        bool_t m_talkToken;
        bool_t m_blockRej;
        uint_t m_txMsgCount;
        uint_t m_pendingTxCount;
        uint_t m_iFramesSinceAckSent;
        uint64_t m_timeout;
        uint64_t m_pollFlagTxTime;
        uint_t m_ctrlFrameSize;
        uint8_t m_ctrlFrameBuf[256];
        std::vector<uint8_t> m_multiFrameBuf;
        bool_t m_multiFrameBuffering;

        void reset();
        void sendSFrame(IslHdlcPacket::SframeCode code);
        void sendUFrame(IslHdlcPacket::UframeCode code, uint8_t address, void* data, uint_t size);
        void processIFrame(const IslHdlcPacket& packet);
        void processSFrame(const IslHdlcPacket& packet);
        void processUFrame(const IslHdlcPacket& packet);
        void processAck(const IslHdlcPacket::Header& hdr);
        bool_t transmitFrame(uint8_t* frame, uint_t size);
    };
}
//--------------------------------------------------------------------------------------------------
#endif
