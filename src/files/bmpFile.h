#ifndef BMP_FILE_H
#define BMP_FILE_H

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"
#include <string>

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    namespace BmpFile
    {
        bool_t save(const std::string& filename, const uint32_t* image, uint_t bpp, uint_t width, uint_t height);
    };
}
//--------------------------------------------------------------------------------------------------
#endif
