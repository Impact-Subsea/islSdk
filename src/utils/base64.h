#ifndef BASE64_H_
#define BASE64_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"
#include <string>

//--------------------------------------- Public Constants -----------------------------------------

//----------------------------------------- Public Types ------------------------------------------

//---------------------------------- Public Function Prototypes -----------------------------------
namespace IslSdk
{
    namespace Base64
    {
        std::string encode(const uint8_t* buf, uint_t size);
        uint_t decode(const std::string& str, uint8_t* data, uint_t size);
    }
}
//--------------------------------------------------------------------------------------------------
#endif
