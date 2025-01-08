#ifndef QUATERNION_H_
#define QUATERNION_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"
#include "maths/vector.h"
#include "maths/matrix.h"
#include "maths/maths.h"

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    namespace Math
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

        class Quaternion : public Vector<4>
        {
        public:
            Quaternion();
            Quaternion(real_t w, real_t x, real_t y, real_t z);
            Quaternion(const Vector<4>& m_data);
            Quaternion(const Vector3& axis, real_t angle);
            Quaternion(const Vector3& rotation);
            Quaternion(const Matrix3x3& m);
            Quaternion operator*(const Quaternion& q) const;
            Vector3 operator*(const Vector3& v) const;
            Quaternion& operator*=(const Quaternion& q);
            Quaternion conjugate() const;
            Matrix3x3 toMatrix() const;
            real_t toAxisAngle(Vector3& axis) const;
            EulerAngles toEulerAngles(real_t headingOffsetRad = 0) const;
            real_t angleBetween(const Quaternion& q, const Vector3& about, bool_t earthFrame) const;
            real_t getRotationAbout(const Quaternion& q, const Vector3& about) const;
        };
    }
}
//--------------------------------------------------------------------------------------------------
#endif
