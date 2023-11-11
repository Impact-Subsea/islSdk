
#ifndef MATHS_H_
#define MATHS_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"
#include <cmath>
#include <stdlib.h>
#include <float.h>
#include <random>

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    namespace Math
    {
        using std::abs;
        using std::sqrt;
        using std::cbrt;
        using std::pow;
        using std::log;
        using std::log10;
        using std::floor;
        using std::ceil;
        using std::round;
        using std::trunc;
        using std::fmod;
        using std::modf;
        using std::sin;
        using std::cos;
        using std::tan;
        using std::asin;
        using std::acos;
        using std::atan;
        using std::atan2;
        using std::max;
        using std::min;
        using std::modf;
        using std::isnan;
        using std::isinf;

        constexpr real_t pi = static_cast<real_t>(3.1415926535897932384626433832795);
        constexpr real_t pi2 = pi * static_cast<real_t>(2.0);
        constexpr real_t rootHalf = static_cast<real_t>(0.70710678118654752440084436210485);
        constexpr uint_t maxUint = ~0;
        constexpr int_t maxInt = static_cast<int_t>(maxUint >> 1);
        constexpr int_t minInt = ~maxInt;
        constexpr uint32_t maxUint32 = static_cast<uint32_t>(~0);
        constexpr int32_t maxInt32 = maxUint32 >> 1;
        constexpr int32_t minInt32 = ~maxInt32;

        constexpr real_t degToRad(real_t deg) { return deg * (pi / static_cast<real_t>(180.0)); }
        constexpr real_t radToDeg(real_t rad) { return rad * (static_cast<real_t>(180.0) / pi); }

        static uint_t randomNum(uint_t l = 1, uint_t h = Math::maxUint)
        {
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<uint_t> distrib(l, h);
            return distrib(gen);
        }
    };
}
//--------------------------------------------------------------------------------------------------
#endif
