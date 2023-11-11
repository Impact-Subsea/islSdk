//------------------------------------------ Includes ----------------------------------------------

#include "mem.h"

using namespace IslSdk;

//--------------------------------------------------------------------------------------------------
bool_t Mem::isLittleEndian()
{
    const uint32_t val = 0x44332211;
    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&val);
    return (*ptr == 0x11);
}
//--------------------------------------------------------------------------------------------------
void Mem::endianCopy(void* dst, const void* src, uint_t size)
{
    if (isLittleEndian())
    {
        std::memcpy(dst, src, size);
    }
    else
    {
        for (uint_t i = 0; i < size; i++)
        {
            reinterpret_cast<uint8_t*>(dst)[i] = reinterpret_cast<const uint8_t*>(src)[size - 1 - i];
        }
    }
}
//--------------------------------------------------------------------------------------------------
void Mem::pack16Bit(uint8_t** dst, uint16_t value)
{
    endianCopy(*dst, &value, 2);
    *dst += 2;
}
//--------------------------------------------------------------------------------------------------
void Mem::pack16Bit(uint8_t* dst, uint16_t value)
{
    endianCopy(dst, &value, 2);
}
//--------------------------------------------------------------------------------------------------
void Mem::pack32Bit(uint8_t** dst, uint32_t value)
{
    endianCopy(*dst, &value, 4);
    *dst += 4;
}
//--------------------------------------------------------------------------------------------------
void Mem::pack32Bit(uint8_t* dst, uint32_t value)
{
    endianCopy(dst, &value, 4);
}
//--------------------------------------------------------------------------------------------------
void Mem::pack48Bit(uint8_t** dst, uint64_t value)
{
    endianCopy(*dst, &value, 6);
    *dst += 6;
}
//--------------------------------------------------------------------------------------------------
void Mem::pack48Bit(uint8_t* dst, uint64_t value)
{
    endianCopy(dst, &value, 6);
}
//--------------------------------------------------------------------------------------------------
void Mem::pack64Bit(uint8_t** dst, uint64_t value)
{
    endianCopy(*dst, &value, 8);
    *dst += 8;
}
//--------------------------------------------------------------------------------------------------
void Mem::pack64Bit(uint8_t* dst, uint64_t value)
{
    endianCopy(dst, &value, 8);
}
//--------------------------------------------------------------------------------------------------
void Mem::packFloat32(uint8_t** dst, real_t value)
{
    float32_t float32 = static_cast<float32_t>(value);
    endianCopy(*dst, &float32, 4);
    *dst += 4;
}
//--------------------------------------------------------------------------------------------------
void Mem::packFloat32(uint8_t* dst, real_t value)
{
    float32_t float32 = static_cast<float32_t>(value);
    endianCopy(dst, &float32, 4);
}
//--------------------------------------------------------------------------------------------------
void Mem::packDouble64(uint8_t** dst, real_t value)
{
    double64_t double64 = static_cast<double64_t>(value);
    endianCopy(*dst, &double64, 8);
    *dst += 8;
}
//--------------------------------------------------------------------------------------------------
void Mem::packDouble64(uint8_t* dst, real_t value)
{
    double64_t double64 = static_cast<double64_t>(value);
    endianCopy(dst, &double64, 8);
}
//--------------------------------------------------------------------------------------------------
uint16_t Mem::get16Bit(const uint8_t** data)
{
    uint16_t value;
    endianCopy(&value, *data, 2);
    *data += 2;
    return value;
}
//--------------------------------------------------------------------------------------------------
uint16_t Mem::get16Bit(const uint8_t* data)
{
    uint16_t value;
    endianCopy(&value, data, 2);
    return value;
}
//--------------------------------------------------------------------------------------------------
uint32_t Mem::get32Bit(const uint8_t** data)
{
    uint32_t value;
    endianCopy(&value, *data, 4);
    *data += 4;
    return value;
}
//--------------------------------------------------------------------------------------------------
uint32_t Mem::get32Bit(const uint8_t* data)
{
    uint32_t value;
    endianCopy(&value, data, 4);
    return value;
}
//--------------------------------------------------------------------------------------------------
uint64_t Mem::get48Bit(const uint8_t** data)
{
    uint64_t value = 0;
    endianCopy(&value, *data, 6);
    *data += 6;
    return value;
}
//--------------------------------------------------------------------------------------------------
uint64_t Mem::get48Bit(const uint8_t* data)
{
    uint64_t value = 0;
    endianCopy(&value, data, 6);
    return value;
}
//--------------------------------------------------------------------------------------------------
uint64_t Mem::get64Bit(const uint8_t** data)
{
    uint64_t value;
    endianCopy(&value, *data, 8);
    *data += 8;
    return value;
}
//--------------------------------------------------------------------------------------------------
uint64_t Mem::get64Bit(const uint8_t* data)
{
    uint64_t value;
    endianCopy(&value, data, 8);
    return value;
}
//--------------------------------------------------------------------------------------------------
real_t Mem::getFloat32(const uint8_t** data)
{
    float32_t value = 0;
    endianCopy(&value, *data, 4);
    *data += 4;
    return static_cast<real_t>(value);
}
//--------------------------------------------------------------------------------------------------
real_t Mem::getFloat32(const uint8_t* data)
{
    float32_t value = 0;
    endianCopy(&value, data, 4);
    return static_cast<real_t>(value);
}
//--------------------------------------------------------------------------------------------------
real_t Mem::getDouble64(const uint8_t** data)
{
    double64_t value = 0;
    endianCopy(&value, *data, 8);
    *data += 8;
    return static_cast<real_t>(value);
}
//--------------------------------------------------------------------------------------------------
real_t Mem::getDouble64(const uint8_t* data)
{
    double64_t value = 0;
    endianCopy(&value, data, 8);
    return static_cast<real_t>(value);
}
//--------------------------------------------------------------------------------------------------
