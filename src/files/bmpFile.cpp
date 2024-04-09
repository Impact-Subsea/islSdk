//------------------------------------------ Includes ----------------------------------------------

#include "bmpFile.h"
#include "platform/file.h"
#include "platform/mem.h"
#include <fstream>
#include "platform/debug.h"

using namespace IslSdk;

//--------------------------------------------------------------------------------------------------
bool_t BmpFile::save(const std::string& filename, const uint32_t* image, uint_t bpp, uint_t width, uint_t height)
{
    File::createDir(filename);
    std::ofstream file;
    file.open(filename, std::ofstream::binary | std::ofstream::out | std::ofstream::trunc);

    if (file.is_open())
    {
        const uint_t bitmapHeaderSize = 14;
        const uint_t dibBitmapInfoheaderSize = 40;
        const uint_t dibBitmapV4headerSize = 108;
        const uint_t biPlanes = 1;

        uint8_t header[bitmapHeaderSize + dibBitmapV4headerSize] = {};
        uint_t fileSize;
        uint_t bfOffBits;
        uint_t biSize;
        uint_t rowWidth = ((width * bpp + 31) / 32) * 4;
        uint_t imageSize = rowWidth * height;

        if (bpp == 32)
        {
            fileSize = bitmapHeaderSize + dibBitmapV4headerSize + imageSize;
            bfOffBits = bitmapHeaderSize + dibBitmapV4headerSize;
            biSize = dibBitmapV4headerSize;
        }
        else
        {
            fileSize = bitmapHeaderSize + dibBitmapInfoheaderSize + imageSize;
            bfOffBits = bitmapHeaderSize + dibBitmapInfoheaderSize;
            biSize = dibBitmapInfoheaderSize;
        }

        header[0] = 'B';
        header[1] = 'M';
        Mem::memcpy(&header[2], &fileSize, 4);
        Mem::memcpy(&header[10], &bfOffBits, 4);

        Mem::memcpy(&header[14], &biSize, 4);
        Mem::memcpy(&header[18], &width, 4);
        Mem::memcpy(&header[22], &height, 4);
        Mem::memcpy(&header[26], &biPlanes, 2);
        Mem::memcpy(&header[28], &bpp, 2);
        Mem::memcpy(&header[34], &imageSize, 4);

        if (bpp == 32)
        {
            header[30] = 3;
            header[54] = 0xff;
            header[59] = 0xff;
            header[64] = 0xff;
            header[69] = 0xff;

            header[70] = 0x20;
            header[71] = 0x6e;
            header[72] = 0x69;
            header[73] = 0x57;
        }

        uint8_t* data = new uint8_t[imageSize];

        if (bpp == 24)
        {
            for (uint_t row = 0; row < height; row++)
            {
                uint8_t* ptr = &data[rowWidth * (height - 1 - row)];

                for (uint_t x = 0; x < width; x++)
                {
                    *ptr++ = static_cast<uint8_t>(image[row * width + x] >> 0);
                    *ptr++ = static_cast<uint8_t>(image[row * width + x] >> 8);
                    *ptr++ = static_cast<uint8_t>(image[row * width + x] >> 16);
                }
            }
        }
        else if (bpp == 32)
        {
            for (uint_t row = 0; row < height; row++)
            {
                uint8_t* ptr = &data[rowWidth * (height - 1 - row)];

                for (uint_t x = 0; x < width; x++)
                {
                    *(reinterpret_cast<uint32_t*>(ptr)) = image[row * width + x];
                    ptr += 4;
                }
            }
        }

        file.write(reinterpret_cast<const char*>(&header[0]), bfOffBits);
        file.write(reinterpret_cast<const char*>(data), imageSize);
        file.close();
        delete[] data;
    }
    else
    {
        Debug::log(Debug::Severity::Warning, "Bmp File", "Can't create bmp file: %s", filename.c_str());
        return false;
    }

    return true;
}
//--------------------------------------------------------------------------------------------------
