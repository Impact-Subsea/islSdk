#ifndef AUTODISCOVERY_H_
#define AUTODISCOVERY_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"
#include "comms/connectionMeta.h"

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    class SysPort;

    class AutoDiscovery
    {
    public:
        enum class Type { Isl, Nmea };
        const AutoDiscovery::Type type;
        const uint_t& discoveryCount = m_discoveryCount;

        AutoDiscovery(SysPort& sysPort, AutoDiscovery::Type type) : m_sysPort(sysPort), type(type), m_discoveryCount(0), m_isDiscovering(false) {}
        virtual ~AutoDiscovery() {};
        virtual bool_t run() = 0;
        virtual void stop() = 0;
        void hasDiscovered() { m_discoveryCount++; }

    protected:
        SysPort& m_sysPort;
        uint_t m_discoveryCount;
        bool_t m_isDiscovering;
    };
}
//--------------------------------------------------------------------------------------------------
#endif
