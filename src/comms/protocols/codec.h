#ifndef CODEC_H_
#define CODEC_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"
#include <vector>

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    class Codec
    {
    public:
        enum class Type {None, Cobs, Nmea};

        Codec(uint_t bufSize, Type type);
        virtual ~Codec();
        virtual uint_t decode(const uint8_t* data, uint_t* size) = 0;
        virtual uint_t encode(const uint8_t* data, uint_t size, uint8_t* buf, uint_t bufSize) = 0;
        Type type;
        std::vector<uint8_t> m_frameBuf;
    };
}
//--------------------------------------------------------------------------------------------------
#endif
