#ifndef NMEADEVICE_H_
#define NMEADEVICE_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"
#include "comms/ports/sysPort.h"
#include "types/sigSlot.h"
#include "logging/loggingDevice.h"

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    class NmeaDevice : public LoggingDevice
    {
        friend class NmeaDeviceMgr;

    public:
        typedef std::shared_ptr<NmeaDevice> SharedPtr;
        enum class Type { Unknown, Gps };

        Signal<NmeaDevice&, const std::string&> onError;
        Signal<NmeaDevice&> onDelete;
        const Type type;
        const SysPort::SharedPtr& sysPort = m_sysPort;

        NmeaDevice(Type type);
        NmeaDevice(const SysPort::SharedPtr& sysPort, Type type);
        virtual ~NmeaDevice();
        void newPacketEvent(const uint8_t* data, uint_t size);

    protected:
        virtual bool_t newSentence(const std::string& str) = 0;

    private:
        void removePort(const SysPort& sysPort);
        SysPort::SharedPtr m_sysPort;
    };
}

//--------------------------------------------------------------------------------------------------

#endif
