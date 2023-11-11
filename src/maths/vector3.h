#ifndef VECTOR3_H_
#define VECTOR3_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    class Vector3
    {
    public:
        real_t x;
        real_t y;
        real_t z;
        enum class Axis { xPlus, xMinus, yPlus, yMinus, zPlus, zMinus };

        Vector3();
        Vector3(real_t x, real_t y, real_t z);
        ~Vector3() {}

        static Vector3 add(const Vector3& a, const Vector3& b);
        static Vector3 subtract(const Vector3& a, const Vector3& b);
        static Vector3 cross(const Vector3& a, const Vector3& b);
        static real_t dot(const Vector3& a, const Vector3& b);

        bool_t operator==(const Vector3& v) const;
        bool_t operator!=(const Vector3& v) const;
        Vector3 operator+(const Vector3& v) const;
        Vector3 operator-(const Vector3& v) const;
        Vector3 operator*(real_t value) const;

        Vector3& operator+=(const Vector3& v);
        Vector3& operator-=(const Vector3& v);
        Vector3& operator*=(real_t value);

        void zero();
        real_t magnitude() const;
        real_t magnitudeSq() const;
        Vector3 normalise() const;

        real_t dot(const Vector3& v) const;
        Vector3 cross(const Vector3& v) const;
        Axis findClosestCardinalAxis() const;
        static Vector3 getVectorFromAxis(Vector3::Axis axis);
    };
}
//--------------------------------------------------------------------------------------------------
#endif
