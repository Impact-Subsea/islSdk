#ifndef QUATERNION_H_
#define QUATERNION_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"
#include "maths/vector3.h"
#include "maths/matrix3x3.h"
#include "maths/maths.h"

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    struct EulerAngles
    {
        real_t heading;
        real_t pitch;
        real_t roll;
        EulerAngles() : heading(0), pitch(0), roll(0) {}
        EulerAngles& radToDeg()
        {
		    heading = Math::radToDeg(heading);
			pitch = Math::radToDeg(pitch);
			roll = Math::radToDeg(roll);
            return *this;
        }
    };

    class Quaternion
    {
    public:
        real_t w;
        real_t x;
        real_t y;
        real_t z;

        Quaternion();
        Quaternion(real_t w, real_t x, real_t y, real_t z);
        Quaternion(const Vector3& axis, real_t angle);
        Quaternion(const Vector3& rotation);
        Quaternion(const Matrix3x3& m);
        ~Quaternion() {}
        Quaternion operator+(const Quaternion& q) const;
        Quaternion operator*(const Quaternion& q) const;
        Vector3 operator*(const Vector3& v) const;
        Quaternion& operator+=(const Quaternion& q);
        Quaternion& operator*=(const Quaternion& q);
        Quaternion conjugate() const;
        Quaternion normalise() const;
        real_t magnitude() const;
        Matrix3x3 toMatrix() const;
        real_t toAxisAngle(Vector3& axis) const;
        EulerAngles toEulerAngles(real_t headingOffsetRad=0) const;
        real_t angleBetween(const Quaternion& q, const Vector3& about, bool_t earthFrame) const;
        real_t getRotationAbout(const Quaternion& q, const Vector3& about) const;
        void getDownAndEulerOffsets(Vector3& down, EulerAngles& euler) const;
    };
}
//--------------------------------------------------------------------------------------------------
#endif
