#ifndef CRC_H_
#define CRC_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    uint32_t crc32(uint32_t seed, const void* data, uint_t length);
    uint16_t crc16(uint16_t seed, const void* data, uint_t length);
}
//--------------------------------------------------------------------------------------------------
#endif
