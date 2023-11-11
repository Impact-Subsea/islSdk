#ifndef FILE_H_
#define FILE_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"
#include <string>

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    namespace File
    {
        std::string getDir(const std::string& path);
        bool_t createDir(const std::string& path);
        bool_t deleteFile(const std::string& filename);
    };
}
//--------------------------------------------------------------------------------------------------
#endif
