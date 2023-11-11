#ifndef ISLDEVICEDISCOVERY_H_
#define ISLDEVICEDISCOVERY_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"
#include "comms/discovery/autoDiscovery.h"
#include "comms/connectionMeta.h"
#include <list>

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    class IslDeviceDiscovery : public AutoDiscovery
    {
    public:
        IslDeviceDiscovery(SysPort& sysPort);
        ~IslDeviceDiscovery();
        bool_t run() override;
        void stop() override;
        void addTask(uint16_t pid, uint16_t pn, uint16_t sn, const ConnectionMeta& meta, uint_t timeoutMs, uint_t count);
        void removeTask(uint16_t pid, uint16_t pn, uint16_t sn);

    private:
        struct DiscoveryParam
        {
            uint16_t pid;
            uint16_t pn;
            uint16_t sn;
            ConnectionMeta meta;
            uint_t timeoutMs;
            uint_t count;

            DiscoveryParam(uint16_t pid, uint16_t pn, uint16_t sn, const ConnectionMeta& meta, uint_t timeoutMs, uint_t count) : pid(pid), pn(pn), sn(sn), meta(meta), timeoutMs(timeoutMs), count(count) {}
        };

        std::list<DiscoveryParam> m_discoveryParam;
        uint64_t m_timeoutMs;
    };
}
//--------------------------------------------------------------------------------------------------
#endif
