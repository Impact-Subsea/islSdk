
#ifndef SDKTYPES_H_
#define SDKTYPES_H_

//------------------------------------------ Includes ----------------------------------------------

#include <stdint.h>
#include <stdbool.h>

//--------------------------------------- Class Definition -----------------------------------------

typedef bool bool_t;
typedef float float_t;
typedef double double_t;
typedef float float32_t;
typedef double double64_t;

#if defined(SYSTEM_32BIT)
    typedef uint32_t uint_t;
    typedef int32_t int_t;
    typedef float real_t;
#elif defined(SYSTEM_64BIT)
    typedef uint64_t uint_t;
    typedef int64_t int_t;
    typedef double real_t;
#else
#error "Unknown system type. Define SYSTEM_32BIT or SYSTEM_64BIT"
#endif

namespace IslSdk
{
    template<typename T, uint_t N> constexpr uint_t countof(T(&)[N]) { return N; }

    struct Point
    {
        real_t x;
        real_t y;
        Point() : x(0), y(0) {}
        Point(real_t x, real_t y) : x(x), y(y) {}
    };
}
//--------------------------------------------------------------------------------------------------
#endif
