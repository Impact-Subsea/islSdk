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
    real_t cosH = Math::cos(Math::degToRad(heading));
    real_t sinH = Math::sin(Math::degToRad(heading));
    real_t cosP = Math::cos(Math::degToRad(pitch));
    real_t sinP = Math::sin(Math::degToRad(pitch));
    real_t cosR = Math::cos(Math::degToRad(roll));
    real_t sinR = Math::sin(Math::degToRad(roll));

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
    *this = fromDownAndForward(down, forward);
}
//--------------------------------------------------------------------------------------------------
Matrix3x3::Matrix3x3(const Vector3& axis, real_t angle)
{
	real_t cosA = Math::cos(angle);
	real_t sinA = Math::sin(angle);
	real_t oneMinusCosA = 1.0 - cosA;

	m[0][0] = cosA + axis.x * axis.x * oneMinusCosA;
	m[0][1] = axis.x * axis.y * oneMinusCosA - axis.z * sinA;
	m[0][2] = axis.x * axis.z * oneMinusCosA + axis.y * sinA;
	m[1][0] = axis.y * axis.x * oneMinusCosA + axis.z * sinA;
	m[1][1] = cosA + axis.y * axis.y * oneMinusCosA;
	m[1][2] = axis.y * axis.z * oneMinusCosA - axis.x * sinA;
	m[2][0] = axis.z * axis.x * oneMinusCosA - axis.y * sinA;
	m[2][1] = axis.z * axis.y * oneMinusCosA + axis.x * sinA;
	m[2][2] = cosA + axis.z * axis.z * oneMinusCosA;
}
//--------------------------------------------------------------------------------------------------
Matrix3x3 Matrix3x3::fromDownAndForward(const Vector3& down, const Vector3& forward)
{
    Matrix3x3 m;
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

    return m;
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
real_t Matrix3x3::determinant() const
{
    return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
           m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
           m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
}
//--------------------------------------------------------------------------------------------------
Matrix3x3 Matrix3x3::inverseSymmetric() const
{
    real_t fB11B22mB12B12 = m[1][1] * m[2][2] - m[1][2] * m[1][2];
    real_t fB12B02mB01B22 = m[1][2] * m[0][2] - m[0][1] * m[2][2];
    real_t fB01B12mB11B02 = m[0][1] * m[1][2] - m[1][1] * m[0][2];
    Matrix3x3 A;

    // set det to the determinant of the matrix
    real_t det = m[0][0] * fB11B22mB12B12 + m[0][1] * fB12B02mB01B22 + m[0][2] * fB01B12mB11B02;

    // set A to the inverse of m for any determinant except zero
    if (det != 0.0)
    {
        det = 1.0 / det;
        A[0][0] = fB11B22mB12B12 * det;
        A[1][0] = A[0][1] = fB12B02mB01B22 * det;
        A[2][0] = A[0][2] = fB01B12mB11B02 * det;
        A[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[0][2]) * det;
        A[2][1] = A[1][2] = (m[0][2] * m[0][1] - m[0][0] * m[1][2]) * det;
        A[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[0][1]) * det;
    }

    return A;
}
//--------------------------------------------------------------------------------------------------
std::array<real_t, 3>& Matrix3x3::operator[](uint_t index)
{
    return m[index];
}
//--------------------------------------------------------------------------------------------------
const std::array<real_t, 3>& Matrix3x3::operator[](uint_t index) const
{
    return m[index];
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
Matrix3x3 Matrix3x3::operator*(real_t scaler) const
{
    Matrix3x3 ret;

    for (uint_t i = 0; i < 3; i++)
    {
        for (uint_t j = 0; j < 3; j++)
        {
            ret[i][j] = m[i][j] * scaler;
        }
    }
    return ret;
}
//--------------------------------------------------------------------------------------------------
Matrix3x3 Matrix3x3::operator*=(real_t scaler)
{
    for (uint_t i = 0; i < 3; i++)
    {
        for (uint_t j = 0; j < 3; j++)
        {
             m[i][j] *= scaler;
        }
    }
    return *this;
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
