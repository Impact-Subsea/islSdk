//------------------------------------------ Includes ----------------------------------------------

#include "file.h"
#include <filesystem>

using namespace IslSdk;

//--------------------------------------------------------------------------------------------------
std::string File::getDir(const std::string& path)
{
    std::filesystem::path dir(path);
    return dir.parent_path().string();
}
//--------------------------------------------------------------------------------------------------
bool_t File::createDir(const std::string& path)
{
    if (path.empty()) return false;

    std::filesystem::path dir(path);
    dir = dir.parent_path();

    try {
        std::filesystem::create_directories(dir);
    }
    catch (std::exception&) {
        return false;
    }

    return std::filesystem::exists(dir);
}
//--------------------------------------------------------------------------------------------------
bool_t File::deleteFile(const std::string& filename)
{
    if (std::filesystem::exists(filename))
    {
        try {
            return std::filesystem::remove(filename);
        }
        catch (std::exception&) {
            return false;
        }
    }

    return false;
}
//--------------------------------------------------------------------------------------------------
