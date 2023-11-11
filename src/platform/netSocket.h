#ifndef NETSOCKET_H_
#define NETSOCKET_H_

//------------------------------------------ Includes ----------------------------------------------

#ifdef OS_WINDOWS
    #include "platform/windows/netSocket.h"
#elif OS_UNIX
    #include "platform/unix/netSocket.h"
#else
    #error "Unsupported platform. Define OS_WINDOWS or OS_UNIX"
#endif

//--------------------------------------- Class Definition -----------------------------------------

//--------------------------------------------------------------------------------------------------
#endif
