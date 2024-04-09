#ifndef PROTOCOLDDEBUGGER_H_
#define PROTOCOLDDEBUGGER_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"
#include "comms/ports/sysPort.h"
#include "comms/protocols/cobs.h"
#include <string>

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    class ProtocolDebugger                /// A class for displaying and debugging the comms protocol between the host and device.
    {
    public:
        ProtocolDebugger(const std::string& name);
        ~ProtocolDebugger();
        void monitorPort(const SysPort::SharedPtr& port);
        bool_t m_showPayload;

    private:
        Slot<SysPort&, const ConstBuffer&> slotOnRxData{ this, &ProtocolDebugger::rxData };
        Slot<SysPort&, const ConstBuffer&> slotOnTxData{ this, &ProtocolDebugger::txData };

        void rxData(SysPort& port, const ConstBuffer& buf);
        void txData(SysPort& port, const ConstBuffer& buf);
        void processBytes(SysPort& port, const uint8_t* data, uint_t size, bool_t isTx);
        void decodeFrame(const uint8_t* frame, uint_t size, uint_t totalByteCount, bool_t isRx);
        SysPort::SharedPtr m_port;
        const std::string m_name;
        uint64_t m_startTimeMs;
        uint_t m_txProcessedBytes;
        uint_t m_rxProcessedBytes;
        Cobs m_txCodec{1080};
        Cobs m_rxCodec{1080};
    };
}

//--------------------------------------------------------------------------------------------------
#endif
