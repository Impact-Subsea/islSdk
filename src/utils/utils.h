#ifndef UTILS_H_
#define UTILS_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"
#include <vector>
#include <string>

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    namespace Utils
    {
        uint32_t ipToUint(uint8_t a, uint8_t b, uint8_t c, uint8_t d);
        template<typename T> bool_t checkVar(T val, T min, T max, std::vector<std::string>& msgs, const std::string msg)
        {
            bool_t error = val < min || val > max;
            if (error)
            {
                msgs.push_back(msg);
            }
            return error;
        }
    }
}
//--------------------------------------------------------------------------------------------------
#endif
