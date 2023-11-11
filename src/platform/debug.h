#ifndef DEBUG_H_
#define DEBUG_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    #define FMT_I(var) static_cast<int_t>(var)
    #define FMT_U(var) static_cast<uint_t>(var)
    #define FMT_X(var) static_cast<uint_t>(var)
    #define FMT_F(var) static_cast<real_t>(var)

#ifdef OS_WINDOWS
    #define NEW_LINE "\r\n"
#else
    #define NEW_LINE "\r\n"
#endif

#ifdef DEBUG_ON
    #define debugLog(unit, fmt, ...) Debug::log(Debug::Severity::Debug, unit, fmt , ##__VA_ARGS__)
#else
    #define debugLog(id, fmt, ...)
#endif

    class Debug
    {
    public:
        enum class Severity { Error, Warning, Notice, Info, Debug, None };
        static Severity reportLevel;
        static void log(Severity level, const char* unit, const char* msg, ...);
    };

//--------------------------------------------------------------------------------------------------
}
#endif
