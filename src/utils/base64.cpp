//------------------------------------------ Includes ----------------------------------------------

#include "base64.h"
#include "utils/stringUtils.h"


using namespace IslSdk;

//--------------------------------------------------------------------------------------------------
std::string Base64::encode(const uint8_t* buf, uint_t size)
{
    std::string str;

    constexpr uint8_t lut[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    const uint8_t* in = buf;
    const uint8_t* end = buf + size;

    while (end - in >= 3)
    {
        str += lut[in[0] >> 2];
        str += lut[((in[0] & 0x03) << 4) | (in[1] >> 4)];
        str += lut[((in[1] & 0x0f) << 2) | (in[2] >> 6)];
        str += lut[in[2] & 0x3f];
        in += 3;
    }

    if (end - in)
    {
        str += lut[in[0] >> 2];
        if (end - in == 1)
        {
            str += lut[(in[0] & 0x03) << 4];
            str += '=';
        }
        else
        {
            str += lut[((in[0] & 0x03) << 4) | (in[1] >> 4)];
            str += lut[(in[1] & 0x0f) << 2];
        }
        str += '=';
    }

    return str;
}
//--------------------------------------------------------------------------------------------------
uint_t Base64::decode(const std::string& str, uint8_t* data, uint_t size)
{
    constexpr uint8_t lut[256] = { 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 0,
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 62, 63, 62, 62, 63, 52, 53, 54, 55,
    56, 57, 58, 59, 60, 61,  0,  0,  0,  0,  0,  0,  0,  0,  1,  2,  3,  4,  5,  6,
    7,  8,  9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25,  0,
    0,  0,  0, 63,  0, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40,
    41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51 };

    uint_t len = str.size();
    while (str[len - 1] == '=')
    {
        len--;
    }

    uint_t last = len & ~0x03;
    uint_t extra = len & 0x03;
    uint_t dataSize = last / 4 * 3;
    if (extra)
    {
        dataSize += (extra - 1);
    }

    if (size >= dataSize)
    {
        uint_t temp;

        for (uint_t i = 0; i < last; i += 4)
        {
            temp = static_cast<uint_t>(lut[str[i]]) << 18 | static_cast<uint_t>(lut[str[i + 1]]) << 12 | static_cast<uint_t>(lut[str[i + 2]]) << 6 | lut[str[i + 3]];
            *data++ = static_cast<uint8_t>((temp >> 16));
            *data++ = static_cast<uint8_t>((temp >> 8));
            *data++ = static_cast<uint8_t>(temp);
        }

        if (extra >= 2)
        {
            temp = static_cast<uint_t>(lut[str[last]]) << 18 | static_cast<uint_t>(lut[str[last + 1]]) << 12;
            *data++ = static_cast<uint8_t>((temp >> 16));

            if (extra >= 3)
            {
                temp |= static_cast<uint_t>(lut[str[last + 2]]) << 6;
                *data++ = static_cast<uint8_t>((temp >> 8));
            }
        }
        return dataSize;
    }
    return 0;
}
//--------------------------------------------------------------------------------------------------
