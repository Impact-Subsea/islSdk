#ifndef TIMEUTILS_H_
#define TIMEUTILS_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    namespace Time
    {
        int64_t getCorrectionMs();
        void resetCorrection();
        uint64_t getTimeMs();
        int64_t getSystemTimeMs();
        void set(int_t year, int_t month, int_t day, int_t hour, int_t minute, real_t second);
    }
}
//--------------------------------------------------------------------------------------------------

#endif
