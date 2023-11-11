//------------------------------------------ Includes ----------------------------------------------

#include "debug.h"
#include <stdarg.h>
#include <stdio.h>

using namespace IslSdk;

Debug::Severity Debug::reportLevel = Debug::Severity::Debug;

//--------------------------------------------------------------------------------------------------
void Debug::log(Severity level, const char* unit, const char* msg, ...)
{
    if (level == Debug::Severity::Debug)
    {
        //if (unit == "IslDeviceDiscovery") return;
        //if (unit == "NmeaDeviceDiscovery") return;
        //if (unit == "SysPort") return;
        //if (unit == "IslHdlc") return;
        //if (unit == "Device") return;
    }

    if (level <= Debug::reportLevel || level == Debug::Severity::None)
    {
        const char* severityStr[] = { "\033[31mError", "\033[33mWarning", "\033[36mNotice", "\033[32mInfo", "\033[35mDebug", "\033[36m" };
        va_list argList;

        printf("%s \033[90m%s %.5s", severityStr[static_cast<uint_t>(level)], unit, severityStr[static_cast<uint_t>(level)]);
        va_start(argList, msg);
        vprintf(msg, argList);
        va_end(argList);
        printf("\033[39m" NEW_LINE);
    }
}
//--------------------------------------------------------------------------------------------------
