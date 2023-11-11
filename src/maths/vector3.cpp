//------------------------------------------ Includes ----------------------------------------------

#include "maths/vector3.h"
#include <cmath>

using namespace IslSdk;

//--------------------------------------------------------------------------------------------------
Vector3::Vector3() : x(0), y(0), z(0)
{
}
//--------------------------------------------------------------------------------------------------
Vector3::Vector3(real_t x, real_t y, real_t z) : x(x), y(y), z(z)
{
}
//--------------------------------------------------------------------------------------------------
Vector3 Vector3::add(const Vector3& a, const Vector3& b)
{
    return Vector3(a.x + b.x, a.y + b.y, a.z + b.z);
}
//--------------------------------------------------------------------------------------------------
Vector3 Vector3::subtract(const Vector3& a, const Vector3& b)
{
    return Vector3(a.x - b.x, a.y - b.y, a.z - b.z);
}
//--------------------------------------------------------------------------------------------------
Vector3 Vector3::cross(const Vector3& a, const Vector3& b)
{
    return Vector3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}
//--------------------------------------------------------------------------------------------------
real_t Vector3::dot(const Vector3& a, const Vector3& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
//--------------------------------------------------------------------------------------------------
bool_t Vector3::operator==(const Vector3& v) const
{
    return !(*this != v);
}
//--------------------------------------------------------------------------------------------------
bool_t Vector3::operator!=(const Vector3& v) const
{
    return x != v.x || y != v.y || z != v.z;
}
//--------------------------------------------------------------------------------------------------
Vector3 Vector3::operator+(const Vector3& v) const
{
    return Vector3(x + v.x, y + v.y, z + v.z);
}
//--------------------------------------------------------------------------------------------------
Vector3 Vector3::operator-(const Vector3& v) const
{
    return Vector3(x - v.x, y - v.y, z - v.z);
}
//--------------------------------------------------------------------------------------------------
Vector3 Vector3::operator*(real_t s) const
{
    return Vector3(x * s, y * s, z * s);
}
//--------------------------------------------------------------------------------------------------
Vector3& Vector3::operator+=(const Vector3& v)
{
    x += v.x;
    y += v.y;
    z += v.z;
    return *this;
}
//--------------------------------------------------------------------------------------------------
Vector3& Vector3::operator-=(const Vector3& v)
{
    x -= v.x;
    y -= v.y;
    z -= v.z;
    return *this;
}
//--------------------------------------------------------------------------------------------------
Vector3& Vector3::operator*=(real_t s)
{
    x *= s;
    y *= s;
    z *= s;
    return *this;
}
//--------------------------------------------------------------------------------------------------
void Vector3::zero()
{
    x = 0;
    y = 0;
    z = 0;
}
//--------------------------------------------------------------------------------------------------
real_t Vector3::magnitude() const
{
    return sqrt(x * x + y * y + z * z);
}
//--------------------------------------------------------------------------------------------------
real_t Vector3::magnitudeSq() const
{
    return x * x + y * y + z * z;
}
//--------------------------------------------------------------------------------------------------
Vector3 Vector3::normalise() const
{
    real_t norm = magnitude();

    if (norm != (real_t)0.0)
    {
        norm = (real_t)1.0 / norm;
    }
    return Vector3(x * norm, y * norm, z * norm);
}
//--------------------------------------------------------------------------------------------------
real_t Vector3::dot(const Vector3& v) const
{
    return x * v.x + y * v.y + z * v.z;
}
//--------------------------------------------------------------------------------------------------
Vector3 Vector3::cross(const Vector3& v) const
{
    return Vector3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
}
//--------------------------------------------------------------------------------------------------
Vector3::Axis Vector3::findClosestCardinalAxis() const
{
    Axis axis;

    if (abs(x) > abs(y) && abs(x) > abs(y))
    {
        axis = x >= 0.0 ? Axis::xPlus : Axis::xMinus;
    }
    else if (abs(y) > abs(x) && abs(y) > abs(z))
    {
        axis = y >= 0.0 ? Axis::yPlus : Axis::yMinus;
    }
    else
    {
        axis = z >= 0.0 ? Axis::zPlus : Axis::zMinus;
    }
    return axis;
}
//--------------------------------------------------------------------------------------------------
Vector3 Vector3::getVectorFromAxis(Vector3::Axis axis)
{
    real_t v[3] = { 0 };
    uint_t idx = (uint_t)axis;

    v[idx / 2] = idx & 1 ? -1.0 : 1.0;

    return Vector3(v[0], v[1], v[2]);
}
//--------------------------------------------------------------------------------------------------
