#ifndef MATRIX3X3_H_
#define MATRIX3X3_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"
#include "maths/vector3.h"

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    class Matrix3x3
    {
    public:
        real_t m[3][3];
        Matrix3x3();
        Matrix3x3(real_t heading, real_t pitch, real_t roll);
        Matrix3x3(const Vector3& down, const Vector3& forward);
        ~Matrix3x3() {}
        void identity();
        Matrix3x3 transpose() const;
        bool_t operator==(const Matrix3x3& rm) const;
        bool_t operator!=(const Matrix3x3& rm) const;
        Matrix3x3 operator*(const Matrix3x3& rm) const;
        Vector3 operator*(const Vector3& v) const;
    };
}
//--------------------------------------------------------------------------------------------------
#endif
