//------------------------------------------ Includes ----------------------------------------------

#include "maths/matrix.h"
#include "maths/maths.h"

using namespace IslSdk::Math;

//--------------------------------------------------------------------------------------------------
Matrix3x3::Matrix3x3(real_t heading, real_t pitch, real_t roll)
{
	real_t cosH = Math::cos(Math::degToRad(heading));
	real_t sinH = Math::sin(Math::degToRad(heading));
	real_t cosP = Math::cos(Math::degToRad(pitch));
	real_t sinP = Math::sin(Math::degToRad(pitch));
	real_t cosR = Math::cos(Math::degToRad(roll));
	real_t sinR = Math::sin(Math::degToRad(roll));

	m_data[0][0] = cosH * cosP;
	m_data[0][1] = cosH * sinP * sinR - sinH * cosR;
	m_data[0][2] = cosH * sinP * cosR + sinH * sinR;
	m_data[1][0] = sinH * cosP;
	m_data[1][1] = sinH * sinP * sinR + cosH * cosR;
	m_data[1][2] = sinH * sinP * cosR - cosH * sinR;
	m_data[2][0] = -sinP;
	m_data[2][1] = cosP * sinR;
	m_data[2][2] = cosP * cosR;
}
//--------------------------------------------------------------------------------------------------
Matrix3x3 Matrix3x3::fromDownAndForward(const Vector3& down, const Vector3& forward)
{
	Matrix3x3 m;
	Vector3 gravity = down.normalise();
	Vector3 east = gravity.cross(forward).normalise();
	Vector3 north = east.cross(gravity).normalise();

	// First row vector to North
	m.m_data[0][0] = north.x;
	m.m_data[0][1] = north.y;
	m.m_data[0][2] = north.z;

	// Second row vector to East
	m.m_data[1][0] = east.x;
	m.m_data[1][1] = east.y;
	m.m_data[1][2] = east.z;

	// Third row vector to Gravity
	m.m_data[2][0] = gravity.x;
	m.m_data[2][1] = gravity.y;
	m.m_data[2][2] = gravity.z;

	return m;
}
//--------------------------------------------------------------------------------------------------
Matrix3x3 Matrix3x3::rodrigues(const Vector3& axis, real_t cosA)
{
	Matrix3x3 m;
	real_t sinA = Math::sqrt(1.0 - cosA * cosA);
	real_t oneMinusCosA = 1.0 - cosA;
	Vector3 axisNorm = axis.normalise();

	m.m_data[0][0] = cosA + axisNorm.x * axisNorm.x * oneMinusCosA;
	m.m_data[0][1] = axisNorm.x * axisNorm.y * oneMinusCosA - axisNorm.z * sinA;
	m.m_data[0][2] = axisNorm.x * axisNorm.z * oneMinusCosA + axisNorm.y * sinA;
	m.m_data[1][0] = axisNorm.y * axisNorm.x * oneMinusCosA + axisNorm.z * sinA;
	m.m_data[1][1] = cosA + axisNorm.y * axisNorm.y * oneMinusCosA;
	m.m_data[1][2] = axisNorm.y * axisNorm.z * oneMinusCosA - axisNorm.x * sinA;
	m.m_data[2][0] = axisNorm.z * axisNorm.x * oneMinusCosA - axisNorm.y * sinA;
	m.m_data[2][1] = axisNorm.z * axisNorm.y * oneMinusCosA + axisNorm.x * sinA;
	m.m_data[2][2] = cosA + axisNorm.z * axisNorm.z * oneMinusCosA;

	return m;
}
//--------------------------------------------------------------------------------------------------
real_t Matrix3x3::determinant() const
{
	return m_data[0][0] * (m_data[1][1] * m_data[2][2] - m_data[1][2] * m_data[2][1]) -
		   m_data[0][1] * (m_data[1][0] * m_data[2][2] - m_data[1][2] * m_data[2][0]) +
		   m_data[0][2] * (m_data[1][0] * m_data[2][1] - m_data[1][1] * m_data[2][0]);
}
//--------------------------------------------------------------------------------------------------
Matrix3x3 Matrix3x3::inverseSymmetric() const
{
	real_t fB11B22mB12B12 = m_data[1][1] * m_data[2][2] - m_data[1][2] * m_data[1][2];
	real_t fB12B02mB01B22 = m_data[1][2] * m_data[0][2] - m_data[0][1] * m_data[2][2];
	real_t fB01B12mB11B02 = m_data[0][1] * m_data[1][2] - m_data[1][1] * m_data[0][2];
	Matrix3x3 A;

	// set det to the determinant of the matrix
	real_t det = m_data[0][0] * fB11B22mB12B12 + m_data[0][1] * fB12B02mB01B22 + m_data[0][2] * fB01B12mB11B02;

	// set A.m_data to the inverse of m_data for any determinant except zero
	if (det != 0.0)
	{
		det = 1.0 / det;
		A.m_data[0][0] = fB11B22mB12B12 * det;
		A.m_data[1][0] = A.m_data[0][1] = fB12B02mB01B22 * det;
		A.m_data[2][0] = A.m_data[0][2] = fB01B12mB11B02 * det;
		A.m_data[1][1] = (m_data[0][0] * m_data[2][2] - m_data[0][2] * m_data[0][2]) * det;
		A.m_data[2][1] = A.m_data[1][2] = (m_data[0][2] * m_data[0][1] - m_data[0][0] * m_data[1][2]) * det;
		A.m_data[2][2] = (m_data[0][0] * m_data[1][1] - m_data[0][1] * m_data[0][1]) * det;
	}

	return A;
}
//--------------------------------------------------------------------------------------------------