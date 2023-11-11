#ifndef NMEADISCOVERY_H_
#define NMEADISCOVERY_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"
#include "comms/discovery/autoDiscovery.h"
#include "comms/ports/sysPort.h"
#include <list>

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    class NmeaDiscovery : public AutoDiscovery
    {
    public:
        NmeaDiscovery(SysPort& sysPort);
        ~NmeaDiscovery();
        bool_t run() override;
        void stop() override;
        void addTask(const ConnectionMeta& meta, uint_t timeoutMs, uint_t count);

    private:
        struct DiscoveryParam
        {
            ConnectionMeta meta;
            uint_t timeoutMs;
            uint_t count;

            DiscoveryParam(const ConnectionMeta& meta, uint_t timeoutMs, uint_t count) : meta(meta), timeoutMs(timeoutMs), count(count) {}
        };

        std::list<DiscoveryParam> m_discoveryParam;
        uint64_t m_timeoutMs;
    };
}
//--------------------------------------------------------------------------------------------------
#endif
