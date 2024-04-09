#ifndef SYSPORTSERVICES_H_
#define SYSPORTSERVICES_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"
#include "comms/ports/sysPort.h"
#include "comms/ports/solPort.h"

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    class SysPortServices
    {
    public:
        virtual const std::shared_ptr<SolPort> createSol(const std::string& name, bool_t isTcp, bool_t useTelnet, uint32_t ipAddress, uint16_t port) = 0;
        virtual void deleteSolSysPort(const SysPort::SharedPtr& sysPort) = 0;

    private:
    };
}
//--------------------------------------------------------------------------------------------------
#endif
