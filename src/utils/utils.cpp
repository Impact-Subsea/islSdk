//------------------------------------------ Includes ----------------------------------------------

#include "utils/utils.h"

using namespace IslSdk;

//--------------------------------------------------------------------------------------------------
uint32_t Utils::ipToUint(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{
    return (static_cast<uint32_t>(a) & 0xff) | ((static_cast<uint32_t>(b) << 8) & 0xff00) |
        ((static_cast<uint32_t>(c) << 16) & 0xff0000) | ((static_cast<uint32_t>(d) << 24) & 0xff000000);
}
//---------------------------------------------------------------------------------------------------
