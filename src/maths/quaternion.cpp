//------------------------------------------ Includes ----------------------------------------------

#include "maths/quaternion.h"
#include "maths/maths.h"

using namespace IslSdk::Math;

//--------------------------------------------------------------------------------------------------
Quaternion::Quaternion() : Vector<4>(1,0,0,0)
{
}
//--------------------------------------------------------------------------------------------------
Quaternion::Quaternion(real_t w, real_t x, real_t y, real_t z) : Vector<4>(w, x, y, z)
{
}
//--------------------------------------------------------------------------------------------------
Quaternion::Quaternion(const Vector<4>& other) : Vector<4>(other)
{
}
//--------------------------------------------------------------------------------------------------
Quaternion::Quaternion(const Vector3& axis, real_t angle)
{
    Vector3 axisNorm = axis.normalise();
    real_t s = Math::sin(angle * 0.5);
    w = Math::cos(angle * 0.5);
    x = axisNorm.x * s;
    y = axisNorm.y * s;
    z = axisNorm.z * s;
}
//--------------------------------------------------------------------------------------------------
Quaternion::Quaternion(const Vector3& rotation)
{
    real_t m;
    real_t s;

    m = rotation.magnitude();

    if (m != 0.0)
    {
        s = Math::sin(m * 0.5) / m;
        x = rotation.x * s;
        y = rotation.y * s;
        z = rotation.z * s;
    }
    else
    {
        x = 0;
        y = 0;
        z = 0;
    }

    m = x * x + y * y + z * z;
    if (m <= 1.0)
    {
        w = Math::sqrt(1.0 - m);
    }
    else
    {
        w = 0.0;
    }
}
//--------------------------------------------------------------------------------------------------
Quaternion::Quaternion(const Matrix3x3& rm)
{
    real_t t;

    if (rm[2][2] < 0.0)
    {
        if (rm[0][0] > rm[1][1])
        {
            t = 1.0 + rm[0][0] - rm[1][1] - rm[2][2];
            x = t;
            y = rm[1][0] + rm[0][1];
            z = rm[0][2] + rm[2][0];
            w = rm[2][1] - rm[1][2];
        }
        else
        {
            t = 1.0 - rm[0][0] + rm[1][1] - rm[2][2];
            x = rm[1][0] + rm[0][1];
            y = t;
            z = rm[2][1] + rm[1][2];
            w = rm[0][2] - rm[2][0];
        }
    }
    else
    {
        if (rm[0][0] < -rm[1][1])
        {
            t = 1.0 - rm[0][0] - rm[1][1] + rm[2][2];
            x = rm[0][2] + rm[2][0];
            y = rm[2][1] + rm[1][2];
            z = t;
            w = rm[1][0] - rm[0][1];
        }
        else
        {
            t = 1.0 + rm[0][0] + rm[1][1] + rm[2][2];
            x = rm[2][1] - rm[1][2];
            y = rm[0][2] - rm[2][0];
            z = rm[1][0] - rm[0][1];
            w = t;
        }
    }
    t = 0.5 / Math::sqrt(t);

    w *= t;
    x *= t;
    y *= t;
    z *= t;
}
//--------------------------------------------------------------------------------------------------
Quaternion Quaternion::operator*(const Quaternion& q) const
{
    real_t qW = w * q.w - x * q.x - y * q.y - z * q.z;
    real_t qX = w * q.x + x * q.w + y * q.z - z * q.y;
    real_t qY = w * q.y - x * q.z + y * q.w + z * q.x;
    real_t qZ = w * q.z + x * q.y - y * q.x + z * q.w;

    return Quaternion(qW, qX, qY, qZ);
}
//--------------------------------------------------------------------------------------------------
Vector3 Quaternion::operator*(const Vector3& v) const
{
    Quaternion q = conjugate() * (Quaternion(0, v.x, v.y, v.z) * *this);

    return Vector3(q.x, q.y, q.z);
}
//--------------------------------------------------------------------------------------------------
Quaternion& Quaternion::operator*=(const Quaternion& q)
{
    real_t qW = w * q.w - x * q.x - y * q.y - z * q.z;
    real_t qX = w * q.x + x * q.w + y * q.z - z * q.y;
    real_t qY = w * q.y - x * q.z + y * q.w + z * q.x;
    real_t qZ = w * q.z + x * q.y - y * q.x + z * q.w;

    w = qW;
    x = qX;
    y = qY;
    z = qZ;
    return *this;
}
//--------------------------------------------------------------------------------------------------
Quaternion Quaternion::conjugate() const
{
    return Quaternion(w, -x, -y, -z);
}
//--------------------------------------------------------------------------------------------------
real_t Quaternion::angleBetween(const Quaternion& q, const Vector3& about, bool_t earthFrame) const
{
    Quaternion qDif;

    if (earthFrame)
    {
        qDif = q * conjugate();
    }
    else
    {
        qDif = q.conjugate() * *this;
    }

    qDif.conjugate();

    return getRotationAbout(qDif, about);
}
//--------------------------------------------------------------------------------------------------
real_t Quaternion::getRotationAbout(const Quaternion& q, const Vector3& about) const
{
    real_t dot = q.x * about.x + q.y * about.y + q.z * about.z;
    Quaternion qr(q.w, about.x * dot, about.y * dot, about.z * dot);
    qr = qr.normalise();
    real_t angle = 2.0 * Math::acos(qr.w);

    if (dot < 0.0)
    {
        angle = -angle;
    }

    return angle;
}
//--------------------------------------------------------------------------------------------------
Matrix3x3 Quaternion::toMatrix() const
{
    real_t ww = w * w;
    real_t wx = w * x;
    real_t wy = w * y;
    real_t wz = w * z;
    real_t xy = x * y;
    real_t xz = x * z;
    real_t yz = y * z;
    Matrix3x3 m;

    // First row vector to North
    m[0][0] = 2.0 * (ww + x * x) - 1.0;
    m[0][1] = 2.0 * (xy - wz);
    m[0][2] = 2.0 * (xz + wy);

    // Second row vector to East
    m[1][0] = 2.0 * (xy + wz);
    m[1][1] = 2.0 * (ww + y * y) - 1.0;
    m[1][2] = 2.0 * (yz - wx);

    // Third row vector to Gravity
    m[2][0] = 2.0 * (xz - wy);
    m[2][1] = 2.0 * (yz + wx);
    m[2][2] = 2.0 * (ww + z * z) - 1.0;

    return m;
}
//--------------------------------------------------------------------------------------------------
real_t Quaternion::toAxisAngle(Vector3& axis) const
{
    real_t angle = 2.0 * Math::acos(w);
    real_t s = Math::sqrt(1.0 - w * w);

    axis.x = x;
    axis.y = y;
    axis.z = z;

    if (s != 0.0)
    {
        axis.x /= s;
        axis.y /= s;
        axis.z /= s;
    }

    return angle;
}
//--------------------------------------------------------------------------------------------------
EulerAngles Quaternion::toEulerAngles(real_t headingOffsetRad) const
{
    real_t m20 = 2.0 * (x * z - w * y);
    real_t m21 = 2.0 * (y * z + w * x);
    real_t m22 = 2.0 * (w * w + z * z) - 1.0;

    m20 = Math::clamp(m20, -1.0, 1.0);
    m21 = Math::clamp(m21, -1.0, 1.0);
    m22 = Math::clamp(m22, -1.0, 1.0);

    //Tait-Bryan intrinsic ZYX (HPR) order
    EulerAngles eulerAngles;
    eulerAngles.pitch = Math::asin(-m20);
    eulerAngles.roll = Math::atan2(m21, m22);

    if (eulerAngles.pitch < Math::degToRad(-89.0) || eulerAngles.pitch > Math::degToRad(89.0))                        // Gimbal lock pitch = -90 or +90
    {
        real_t m11 = 2.0 * (w * w + y * y) - 1.0;
        real_t m12 = 2.0 * (y * z - w * x);
        eulerAngles.heading = eulerAngles.pitch < 0.0 ? -Math::atan2(m12, -m11) - eulerAngles.roll : Math::atan2(m12, m11) + eulerAngles.roll;
    }
    else
    {
        real_t m00 = 2.0 * (w * w + x * x) - 1.0;
        real_t m10 = 2.0 * (x * y + w * z);
        eulerAngles.heading = Math::atan2(m10, m00);
    }

    eulerAngles.heading += headingOffsetRad;

    if (eulerAngles.heading < 0.0)
    {
        eulerAngles.heading += Math::pi2;
    }
    else if (eulerAngles.heading >= Math::pi2)
    {
        eulerAngles.heading -= Math::pi2;
    }

    return eulerAngles;
}
//--------------------------------------------------------------------------------------------------
