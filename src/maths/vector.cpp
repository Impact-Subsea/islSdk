//------------------------------------------ Includes ----------------------------------------------

#include "maths/vector.h"

using namespace IslSdk::Math;

//--------------------------------------------------------------------------------------------------
Vector3 Vector3::cross(const Vector<3>& other) const
{
    return Vector3(
        y * other.m_data[2] - z * other.m_data[1],
        z * other.m_data[0] - x * other.m_data[2],
        x * other.m_data[1] - y * other.m_data[0]
    );
}
//--------------------------------------------------------------------------------------------------
Vector3::Axis Vector3::findClosestCardinalAxis() const
{
    Axis axis;

    if (Math::abs(x) > Math::abs(y) && Math::abs(x) > Math::abs(z))
    {
        axis = x >= 0.0 ? Axis::xPlus : Axis::xMinus;
    }
    else if (Math::abs(y) > Math::abs(x) && Math::abs(y) > Math::abs(z))
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
    Math::Vector3 v;
    uint_t idx = static_cast<uint_t>(axis);

    v[idx / 2] = idx & 1 ? -1.0 : 1.0;

    return v;
}
//--------------------------------------------------------------------------------------------------