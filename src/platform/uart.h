#ifndef UART_H_
#define UART_H_

//------------------------------------------ Includes ----------------------------------------------

#ifdef OS_WINDOWS
    #include "platform/windows/uart.h"
#elif OS_UNIX
    #include "platform/unix/uart.h"
#else
    #error "Unsupported platform. Define OS_WINDOWS or OS_UNIX"
#endif

//--------------------------------------------------------------------------------------------------
#endif
