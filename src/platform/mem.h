#ifndef MEM_H_
#define MEM_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"
#include <cstring>

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    namespace Mem
    {
        using std::memcpy;
        using std::memset;
        using std::memcmp;

        const uint_t memoryAligment = sizeof(uint_t);

        bool_t isLittleEndian();
        void endianCopy(void* dst, const void* src, uint_t size);
        void pack16Bit(uint8_t** dst, uint16_t value);
        void pack16Bit(uint8_t* dst, uint16_t value);
        void pack32Bit(uint8_t** dst, uint32_t value);
        void pack32Bit(uint8_t* dst, uint32_t value);
        void pack48Bit(uint8_t** dst, uint64_t value);
        void pack48Bit(uint8_t* dst, uint64_t value);
        void pack64Bit(uint8_t** dst, uint64_t value);
        void pack64Bit(uint8_t* dst, uint64_t value);
        void packFloat32(uint8_t** dst, real_t value);
        void packFloat32(uint8_t* dst, real_t value);
        void packDouble64(uint8_t** dst, real_t value);
        void packDouble64(uint8_t* dst, real_t value);
        uint16_t get16Bit(const uint8_t** data);
        uint16_t get16Bit(const uint8_t* data);
        uint32_t get32Bit(const uint8_t** data);
        uint32_t get32Bit(const uint8_t* data);
        uint64_t get48Bit(const uint8_t** data);
        uint64_t get48Bit(const uint8_t* data);
        uint64_t get64Bit(const uint8_t** data);
        uint64_t get64Bit(const uint8_t* data);
        real_t getFloat32(const uint8_t** data);
        real_t getFloat32(const uint8_t* data);
        real_t getDouble64(const uint8_t** data);
        real_t getDouble64(const uint8_t* data);
    }
}
//--------------------------------------------------------------------------------------------------

#endif
