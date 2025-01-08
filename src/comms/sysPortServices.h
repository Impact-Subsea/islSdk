#ifndef SYSPORTSERVICES_H_
#define SYSPORTSERVICES_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"
#include "comms/ports/sysPort.h"

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    class SysPortServices
    {
    public:
        virtual void addSysPort(const SysPort::SharedPtr& sysPort) = 0;
        virtual void deleteSysPort(const SysPort::SharedPtr& sysPort) = 0;

    private:
    };
}
//--------------------------------------------------------------------------------------------------
#endif
