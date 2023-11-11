//------------------------------------------ Includes ----------------------------------------------

#include "maths/matrix3x3.h"
#include "maths/maths.h"

using namespace IslSdk;

//--------------------------------------------------------------------------------------------------
Matrix3x3::Matrix3x3()
{
    identity();
}
//--------------------------------------------------------------------------------------------------
Matrix3x3::Matrix3x3(real_t heading, real_t pitch, real_t roll)
{
    real_t cosH = cos(Math::degToRad(heading));
    real_t sinH = sin(Math::degToRad(heading));
    real_t cosP = cos(Math::degToRad(pitch));
    real_t sinP = sin(Math::degToRad(pitch));
    real_t cosR = cos(Math::degToRad(roll));
    real_t sinR = sin(Math::degToRad(roll));

    m[0][0] = cosH * cosP;
    m[0][1] = cosH * sinP * sinR - sinH * cosR;
    m[0][2] = cosH * sinP * cosR + sinH * sinR;
    m[1][0] = sinH * cosP;
    m[1][1] = sinH * sinP * sinR + cosH * cosR;
    m[1][2] = sinH * sinP * cosR - cosH * sinR;
    m[2][0] = -sinP;
    m[2][1] = cosP * sinR;
    m[2][2] = cosP * cosR;
}
//--------------------------------------------------------------------------------------------------
Matrix3x3::Matrix3x3(const Vector3& down, const Vector3& forward)
{
    Vector3 gravity = down.normalise();
    Vector3 east = gravity.cross(forward).normalise();
    Vector3 north = east.cross(gravity).normalise();

    // First row vector to North
    m[0][0] = north.x;
    m[0][1] = north.y;
    m[0][2] = north.z;

    // Second row vector to East
    m[1][0] = east.x;
    m[1][1] = east.y;
    m[1][2] = east.z;

    // Third row vector to Gravity
    m[2][0] = gravity.x;
    m[2][1] = gravity.y;
    m[2][2] = gravity.z;
}
//--------------------------------------------------------------------------------------------------
void Matrix3x3::identity()
{
    m[0][0] = static_cast<real_t>(1.0);
    m[0][1] = static_cast<real_t>(0.0);
    m[0][2] = static_cast<real_t>(0.0);
    m[1][0] = static_cast<real_t>(0.0);
    m[1][1] = static_cast<real_t>(1.0);
    m[1][2] = static_cast<real_t>(0.0);
    m[2][0] = static_cast<real_t>(0.0);
    m[2][1] = static_cast<real_t>(0.0);
    m[2][2] = static_cast<real_t>(1.0);
}
//--------------------------------------------------------------------------------------------------
Matrix3x3 Matrix3x3::transpose() const
{
    Matrix3x3 t;

    t.m[0][0] = m[0][0];
    t.m[0][1] = m[1][0];
    t.m[0][2] = m[2][0];

    t.m[1][0] = m[0][1];
    t.m[1][1] = m[1][1];
    t.m[1][2] = m[2][1];

    t.m[2][0] = m[0][2];
    t.m[2][1] = m[1][2];
    t.m[2][2] = m[2][2];

    return t;
}
//--------------------------------------------------------------------------------------------------
bool_t Matrix3x3::operator==(const Matrix3x3& rm) const
{
    return !(*this != rm);
}
//--------------------------------------------------------------------------------------------------
bool_t Matrix3x3::operator!=(const Matrix3x3& rm) const
{
    for (uint_t i = 0; i < 3; i++)
    {
        for (uint_t j = 0; j < 3; j++)
        {
            if (m[i][j] != rm.m[i][j])
            {
                return true;
            }
        }
    }
    return false;
}
//--------------------------------------------------------------------------------------------------
Matrix3x3 Matrix3x3::operator*(const Matrix3x3& rm) const
{
    Matrix3x3 res;

    for (uint_t i = 0; i < 3; i++)
    {
        for (uint_t j = 0; j < 3; j++)
        {
            res.m[i][j] = m[i][0] * rm.m[0][j] + m[i][1] * rm.m[1][j] + m[i][2] * rm.m[2][j];
        }
    }

    return res;
}
//--------------------------------------------------------------------------------------------------
Vector3 Matrix3x3::operator*(const Vector3& v) const
{
    real_t x = m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z;
    real_t y = m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z;
    real_t z = m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z;

    return Vector3(x, y, z);
}
//--------------------------------------------------------------------------------------------------
