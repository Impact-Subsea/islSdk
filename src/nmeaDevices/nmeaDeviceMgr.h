#ifndef NMEADEVICEMGR_H_
#define NMEADEVICEMGR_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"
#include "nmeaDevice.h"
#include "comms/ports/sysPort.h"
#include <list>

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    class NmeaDeviceMgr
    {
        friend class Sdk;
    public:
        const std::list<NmeaDevice::SharedPtr>& deviceList = m_deviceList;
        Signal<const NmeaDevice::SharedPtr&, const SysPort::SharedPtr&, const ConnectionMeta&> onNew;
        NmeaDevice::SharedPtr findBySysPort(const SysPort& sysPort);
        void remove(NmeaDevice& device);

    private:
        std::list<NmeaDevice::SharedPtr> m_deviceList;

        NmeaDeviceMgr();
        NmeaDevice::SharedPtr createDevice(const SysPort::SharedPtr& sysPortPtr, const uint8_t* data, uint_t size);
        void removePortFromAll(const SysPort& sysPort);
    };
}
//--------------------------------------------------------------------------------------------------
#endif
