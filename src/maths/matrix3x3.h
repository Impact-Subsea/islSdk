#ifndef MATRIX3X3_H_
#define MATRIX3X3_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"
#include "maths/vector3.h"
#include <array>

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    class Matrix3x3
    {
    public:
        std::array<std::array<real_t, 3>, 3> m;
        Matrix3x3();
        Matrix3x3(real_t heading, real_t pitch, real_t roll);
        Matrix3x3(const Vector3& down, const Vector3& forward);
        Matrix3x3(const Vector3& axis, real_t angle);
        ~Matrix3x3() {}
        static Matrix3x3 fromDownAndForward(const Vector3& down, const Vector3& forward);
        void identity();
        Matrix3x3 transpose() const;
        real_t determinant() const;
        Matrix3x3 inverseSymmetric() const;
        std::array<real_t, 3>& operator[](uint_t index);
        const std::array<real_t, 3>& operator[](uint_t index) const;
        bool_t operator==(const Matrix3x3& rm) const;
        bool_t operator!=(const Matrix3x3& rm) const;
        Matrix3x3 operator*(const Matrix3x3& rm) const;
        Matrix3x3 operator*(real_t scaler) const;
        Matrix3x3 operator*=(real_t scaler);
        Vector3 operator*(const Vector3& v) const;   
    };
}
//--------------------------------------------------------------------------------------------------
#endif
