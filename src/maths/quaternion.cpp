//------------------------------------------ Includes ----------------------------------------------

#include "maths/quaternion.h"
#include "maths/maths.h"

using namespace IslSdk;

//--------------------------------------------------------------------------------------------------
Quaternion::Quaternion() : w(1), x(0), y(0), z(0)
{
}
//--------------------------------------------------------------------------------------------------
Quaternion::Quaternion(real_t w, real_t x, real_t y, real_t z) : w(w), x(x), y(y), z(z)
{
}
//--------------------------------------------------------------------------------------------------
Quaternion::Quaternion(const Vector3& axis, real_t angle)
{
    real_t s = Math::sin(angle * (real_t)0.5);
    w = Math::cos(angle * (real_t)0.5);
    x = axis.x * s;
    y = axis.y * s;
    z = axis.z * s;
}
//--------------------------------------------------------------------------------------------------
Quaternion::Quaternion(const Vector3& rotation)
{
    real_t m;
    real_t s;

    m = rotation.magnitude();

    if (m != (real_t)0.0)
    {
        s = Math::sin(m * (real_t)0.5) / m;
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
    if (m <= (real_t)1.0)
    {
        w = Math::sqrt((real_t)1.0 - m);
    }
    else
    {
        w = (real_t)0.0;
    }
}
//--------------------------------------------------------------------------------------------------
Quaternion::Quaternion(const Matrix3x3& rm)
{
    real_t t;

    if (rm.m[2][2] < (real_t)0.0)
    {
        if (rm.m[0][0] > rm.m[1][1])
        {
            t = (real_t)1.0 + rm.m[0][0] - rm.m[1][1] - rm.m[2][2];
            x = t;
            y = rm.m[1][0] + rm.m[0][1];
            z = rm.m[0][2] + rm.m[2][0];
            w = rm.m[2][1] - rm.m[1][2];
        }
        else
        {
            t = (real_t)1.0 - rm.m[0][0] + rm.m[1][1] - rm.m[2][2];
            x = rm.m[1][0] + rm.m[0][1];
            y = t;
            z = rm.m[2][1] + rm.m[1][2];
            w = rm.m[0][2] - rm.m[2][0];
        }
    }
    else
    {
        if (rm.m[0][0] < -rm.m[1][1])
        {
            t = (real_t)1.0 - rm.m[0][0] - rm.m[1][1] + rm.m[2][2];
            x = rm.m[0][2] + rm.m[2][0];
            y = rm.m[2][1] + rm.m[1][2];
            z = t;
            w = rm.m[1][0] - rm.m[0][1];
        }
        else
        {
            t = (real_t)1.0 + rm.m[0][0] + rm.m[1][1] + rm.m[2][2];
            x = rm.m[2][1] - rm.m[1][2];
            y = rm.m[0][2] - rm.m[2][0];
            z = rm.m[1][0] - rm.m[0][1];
            w = t;
        }
    }
    t = (real_t)0.5 / Math::sqrt(t);

    w *= t;
    x *= t;
    y *= t;
    z *= t;
}
//--------------------------------------------------------------------------------------------------
Quaternion Quaternion::operator+(const Quaternion& q) const
{
    return Quaternion(w + q.w, x + q.x, y + q.y, z + q.z);
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
Quaternion& Quaternion::operator+=(const Quaternion& q)
{
    w += q.w;
    x += q.x;
    y += q.y;
    z += q.z;
    return *this;
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
Quaternion Quaternion::normalise() const
{
    real_t norm = Math::sqrt(w * w + x * x + y * y + z * z);

    if (norm != (real_t)0.0)
    {
        norm = (real_t)1.0 / norm;
    }
    return Quaternion(w * norm, x * norm, y * norm, z * norm);
}
//--------------------------------------------------------------------------------------------------
real_t Quaternion::magnitude() const
{
    return Math::sqrt(w * w + x * x + y * y + z * z);
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
    Quaternion qr = Quaternion(q.w, about.x * dot, about.y * dot, about.z * dot).normalise();
    real_t angle = (real_t)2.0 * std::acos(qr.w);

    if (dot < (real_t)0.0)
    {
        angle = -angle;
    }

    return angle;
}
//--------------------------------------------------------------------------------------------------
void Quaternion::getDownAndEulerOffsets(Vector3& down, EulerAngles& euler) const
{
    Quaternion qr;
    down.x = (real_t)2.0 * (x * z - w * y);
    down.y = (real_t)2.0 * (y * z + w * x);
    down.z = (real_t)2.0 * (w * w + z * z) - (real_t)1.0;
    down = Vector3::getVectorFromAxis(down.findClosestCardinalAxis());

    if (Math::abs(down.z) > 0.01)
    {
        qr.w = (down.z + 1.0) * 0.5;
        qr.x = qr.w - 1.0;
        qr.y = 0;
        qr.z = 0;
    }
    else
    {
        qr.w = Math::rootHalf;
        qr.x = down.y * -Math::rootHalf;
        qr.y = down.x * Math::rootHalf;
        qr.z = 0;
    }

    qr = *this * qr;
    qr = qr.conjugate();

    real_t m00 = (real_t)2.0 * (qr.w * qr.w + qr.x * qr.x) - (real_t)1.0;
    real_t m10 = (real_t)2.0 * (qr.x * qr.y + qr.w * qr.z);
    real_t m20 = (real_t)2.0 * (qr.x * qr.z - qr.w * qr.y);
    real_t m21 = (real_t)2.0 * (qr.y * qr.z + qr.w * qr.x);

    euler.heading = -Math::atan2(m10, m00);
    euler.pitch = std::asin(m20);
    euler.roll = -std::asin(m21);
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
    m.m[0][0] = (real_t)2.0 * (ww + x * x) - (real_t)1.0;
    m.m[0][1] = (real_t)2.0 * (xy - wz);
    m.m[0][2] = (real_t)2.0 * (xz + wy);

    // Second row vector to East
    m.m[1][0] = (real_t)2.0 * (xy + wz);
    m.m[1][1] = (real_t)2.0 * (ww + y * y) - (real_t)1.0;
    m.m[1][2] = (real_t)2.0 * (yz - wx);

    // Third row vector to Gravity
    m.m[2][0] = (real_t)2.0 * (xz - wy);
    m.m[2][1] = (real_t)2.0 * (yz + wx);
    m.m[2][2] = (real_t)2.0 * (ww + z * z) - (real_t)1.0;

    return m;
}
//--------------------------------------------------------------------------------------------------
real_t Quaternion::toAxisAngle(Vector3& axis) const
{
    real_t angle = (real_t)2.0 * std::acos(w);
    real_t s = Math::sqrt((real_t)1.0 - w * w);

    axis.x = x;
    axis.y = y;
    axis.z = z;

    if (s != (real_t)0.0)
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
    real_t m00;
    real_t m10;
    real_t m11;
    real_t m12;
    real_t m20;
    real_t m21;
    real_t m22;
    EulerAngles eulerAngles;

    m20 = (real_t)2.0 * (x * z - w * y);
    m21 = (real_t)2.0 * (y * z + w * x);
    m22 = (real_t)2.0 * (w * w + z * z) - (real_t)1.0;

    //Tait-Bryan intrinsic ZYX (HPR) order
    eulerAngles.pitch = std::asin(-m20);
    eulerAngles.roll = Math::atan2(m21, m22);

    if (eulerAngles.pitch < Math::degToRad(-89.0) || eulerAngles.pitch > Math::degToRad(89.0))                        // Gimbal lock pitch = -90 or +90
    {
        m11 = (real_t)2.0 * (w * w + y * y) - (real_t)1.0;
        m12 = (real_t)2.0 * (y * z - w * x);
        eulerAngles.heading = eulerAngles.pitch < (real_t)0.0 ? -Math::atan2(m12, -m11) - eulerAngles.roll : Math::atan2(m12, m11) + eulerAngles.roll;
    }
    else
    {
        m00 = (real_t)2.0 * (w * w + x * x) - (real_t)1.0;
        m10 = (real_t)2.0 * (x * y + w * z);
        eulerAngles.heading = Math::atan2(m10, m00);
    }

    eulerAngles.heading += headingOffsetRad;

    if (eulerAngles.heading < (real_t)0.0)
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
