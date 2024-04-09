//------------------------------------------ Includes ----------------------------------------------

#include "ahrs.h"
#include "maths/maths.h"
#include "platform/debug.h"

using namespace IslSdk;

real_t ellipsoidFit(const std::vector<Vector3>& points, Vector3& bias, Matrix3x3& transform);
void eigenCompute(real_t A[][10], real_t eigVal[], real_t eigVec[][10], int_t n);

//==================================================================================================
void Ahrs::setHeading(real_t headingRad)
{
    m_setHeading(&headingRad);
}
//--------------------------------------------------------------------------------------------------
void Ahrs::setHeadingToMag()
{
    m_setHeading(nullptr);
}
//--------------------------------------------------------------------------------------------------
void Ahrs::clearTurnsCount()
{
    m_clearTurnsCount();
}
//==================================================================================================
void GyroSensor::updateCalValues(Vector3& bias)
{
    if (m_bias != bias)
    {
        m_bias = bias;
        onCalChange(*this, m_bias);
    }
}
//--------------------------------------------------------------------------------------------------
void GyroSensor::autoCal()
{
    m_sendCal(m_sensor, nullptr);
}
//--------------------------------------------------------------------------------------------------
void GyroSensor::setCal(Vector3& bias)
{
    updateCalValues(bias);
    m_sendCal(m_sensor, &bias);
}
//==================================================================================================
void AccelSensor::updateCalValues(const Vector3& bias, const Matrix3x3& transform)
{
    if (m_bias != bias || m_transform != transform)
    {
        m_bias = bias;
        m_transform = transform;
        onCalChange(*this, m_bias, m_transform);
    }
}
//--------------------------------------------------------------------------------------------------
void AccelSensor::setCal(const Vector3& bias, const Matrix3x3& transform)
{
    m_sendCal(m_sensor, bias, transform);
    updateCalValues(bias, transform);
}
//--------------------------------------------------------------------------------------------------
void AccelSensor::startCal(uint_t samplesPerAverage, real_t maxAllowableG, real_t degFromCardinal)
{
    m_cal.averageCount = samplesPerAverage;
    m_cal.maxAllowableG = maxAllowableG;
    m_cal.cardinalAxisDeviation = Math::cos(Math::degToRad(degFromCardinal));
    m_cal.average.zero();
    m_cal.count = m_cal.averageCount;
    m_cal.progress = 0;
    m_cal.calData[0].zero();
    m_cal.calData[1].zero();
    m_cal.calData[2].zero();
    m_cal.calData[3].zero();
    m_cal.calData[4].zero();
    m_cal.calData[5].zero();
    onData.connect(slotData);
}
//--------------------------------------------------------------------------------------------------
void AccelSensor::stopCal(bool_t cancel)
{
    Matrix3x3 transform;
    Vector3 bias;

    onData.disconnect(slotData);

    if (!cancel && m_cal.calculate(bias, transform))
    {
        bias += m_bias;
        transform = transform * m_transform;
        setCal(bias, transform);
    }
}
//--------------------------------------------------------------------------------------------------
bool_t AccelSensor::Cal::calculate(Vector3& bias, Matrix3x3& transform)
{
    if (progress == 0x3f)
    {
        bias.x = (calData[static_cast<uint_t>(Vector3::Axis::xMinus)].x + calData[static_cast<uint_t>(Vector3::Axis::xPlus)].x) * 0.5;
        bias.y = (calData[static_cast<uint_t>(Vector3::Axis::yMinus)].y + calData[static_cast<uint_t>(Vector3::Axis::yPlus)].y) * 0.5;
        bias.z = (calData[static_cast<uint_t>(Vector3::Axis::zMinus)].z + calData[static_cast<uint_t>(Vector3::Axis::zPlus)].z) * 0.5;

        Vector3 sensorDown = calData[static_cast<uint_t>(Vector3::Axis::zPlus)].normalise();
        Vector3 sensorFwd = calData[static_cast<uint_t>(Vector3::Axis::xPlus)].normalise();
        Vector3 fwd;
        fwd.z = Math::sqrt(1.0 - sensorDown.z * sensorDown.z);
        real_t scale = Math::sqrt((1.0 - fwd.z * fwd.z) / (sensorFwd.x * sensorFwd.x + sensorFwd.y * sensorFwd.y));
        fwd.x = sensorFwd.x * scale;
        fwd.y = sensorFwd.y * scale;

        transform = Matrix3x3::fromDownAndForward(sensorDown, fwd);

        for (uint_t i = 0; i < 6; i++)
        {
            calData[i] = transform * (calData[i] - bias);
        }

        transform.m[0][0] *= (real_t)2.0 / (calData[static_cast<uint_t>(Vector3::Axis::xPlus)].x - calData[static_cast<uint_t>(Vector3::Axis::xMinus)].x);
        transform.m[1][1] *= (real_t)2.0 / (calData[static_cast<uint_t>(Vector3::Axis::yPlus)].y - calData[static_cast<uint_t>(Vector3::Axis::yMinus)].y);
        transform.m[2][2] *= (real_t)2.0 / (calData[static_cast<uint_t>(Vector3::Axis::zPlus)].z - calData[static_cast<uint_t>(Vector3::Axis::zMinus)].z);

        return true;
    }
    return false;
}
//--------------------------------------------------------------------------------------------------
void AccelSensor::newData(AccelSensor& accel, const Vector3& v)
{
    if (m_cal.count)
    {
        if (Vector3::subtract(m_cal.average, v).magnitude() < m_cal.maxAllowableG)
        {
            m_cal.average += v;
            m_cal.average *= 0.5;
        }
        else
        {
            m_cal.average = v;
            m_cal.count = m_cal.averageCount;
        }
        m_cal.count--;

        if (m_cal.count == 0)
        {
            Vector3::Axis axis = m_cal.average.findClosestCardinalAxis();
            Vector3 cardinal = Vector3::getVectorFromAxis(axis);
            Vector3 norm = m_cal.average.normalise();

            if (norm.dot(cardinal) >= m_cal.cardinalAxisDeviation)
            {
                m_cal.calData[static_cast<uint_t>(axis)] = m_cal.average;
                m_cal.progress |= static_cast<uint_t>(1) << static_cast<uint_t>(axis);
                onCalProgress(*this, axis, m_cal.average, m_cal.progress);
            }

            m_cal.average.zero();
            m_cal.count = m_cal.averageCount;
        }
    }
}
//==================================================================================================
void MagSensor::updateCalValues(const Vector3& bias, const Matrix3x3& transform)
{
    if (m_bias != bias || m_transform != transform)
    {
        m_bias = bias;
        m_transform = transform;
        onCalChange(*this, m_bias, m_transform);
    }
}
//--------------------------------------------------------------------------------------------------
void MagSensor::setCal(const Vector3& bias, const Matrix3x3& transform)
{
    m_sendCal(m_sensor, bias, transform);
    updateCalValues(bias, transform);
}
//--------------------------------------------------------------------------------------------------
void MagSensor::startCal(uint16_t maxSample)
{
    maxSample = Math::min<uint16_t>(maxSample, 10000);

    m_cal.maxCount = maxSample;
    m_cal.count = 0;
    m_cal.surfaceDiv = Math::sqrt(m_cal.maxCount / 2);
    m_cal.points.clear();
    m_cal.surfaceLut.clear();
    m_cal.points.resize(maxSample);
    m_cal.surfaceLut.resize(maxSample, -1);
    onData.connect(slotData);
}
//--------------------------------------------------------------------------------------------------
bool_t MagSensor::stopCal(bool_t cancel, Vector3* biasCorrection, Matrix3x3* transformCorrection)
{
    Matrix3x3 transform;
    Vector3 bias;
    bool_t ok = false;

    m_cal.points.resize(m_cal.count);

    if (!cancel && m_cal.points.size() > 10)
    {
        if (m_cal.calculate(bias, transform))
        {
            if (biasCorrection)
            {
                *biasCorrection = bias;
            }

            if (transformCorrection)
            {
                *transformCorrection = transform;
            }
            bias += m_bias;
            transform = transform * m_transform;
            setCal(bias, transform);
            ok = true;
        }
    }

    onData.disconnect(slotData);

    return ok;
}
//--------------------------------------------------------------------------------------------------
bool_t MagSensor::Cal::calculate(Vector3& bias, Matrix3x3& transform)
{
    ellipsoidFit(points, bias, transform);
    return true;
}
//--------------------------------------------------------------------------------------------------
void MagSensor::newData(MagSensor& mag, const Vector3& magVector)
{
    int_t idx = -1;
    Vector3 v = magVector.normalise();
    real_t x = ((Math::pi / 2.0 + std::asin(v.x)) / Math::pi) * m_cal.surfaceDiv;
    real_t y = ((Math::pi / 2.0 + std::asin(v.y)) / Math::pi) * m_cal.surfaceDiv;

    if (v.z < 0.0)
    {
        x = m_cal.surfaceDiv * 2.0 - x;
    }

    uint_t lutIdx = static_cast<uint_t>(static_cast<uint_t>(y) * m_cal.surfaceDiv * 2.0 + x);

    lutIdx = Math::min<uint_t>(lutIdx, m_cal.maxCount - 1);

    if (m_cal.surfaceLut[lutIdx] >= 0)
    {
        idx = m_cal.surfaceLut[lutIdx];
    }
    else
    {
        m_cal.surfaceLut[lutIdx] = static_cast<int16_t>(m_cal.count);
        idx = static_cast<int_t>(m_cal.count);
        if (m_cal.count < (m_cal.maxCount - 1))
        {
            m_cal.count++;
        }
    }

    real_t qf = Math::max(Math::abs(v.x), Math::max(Math::abs(v.y), Math::abs(v.z)));
    qf = Math::abs(qf * qf - 0.5) * 2.0;

    if (idx >= 0 && idx < static_cast<int_t>(m_cal.maxCount))
    {
        m_cal.points[idx] = magVector;
        onCalProgress(*this, magVector, static_cast<uint_t>(idx), qf);
    }
}
//--------------------------------------------------------------------------------------------------
real_t ellipsoidFit(const std::vector<Vector3>& points, Vector3& bias, Matrix3x3& transform)
{
    real_t matA[10][10] = {};
    real_t vecA[10];
    real_t fitError = 0;

    for (const Vector3& v : points)
    {
        vecA[0] = v.x * v.x;
        vecA[1] = 2.0 * v.x * v.y;
        vecA[2] = 2.0 * v.x * v.z;
        vecA[3] = v.y * v.y;
        vecA[4] = 2.0 * v.y * v.z;
        vecA[5] = v.z * v.z;
        vecA[6] = v.x;
        vecA[7] = v.y;
        vecA[8] = v.z;
        vecA[9] = 0;

        for (uint_t m = 0; m < 9; m++)
        {
            matA[m][9] += vecA[m];
        }

        for (uint_t m = 0; m < 9; m++)
        {
            for (uint_t n = m; n < 9; n++)
            {
                matA[m][n] += vecA[m] * vecA[n];
            }
        }
    }

    matA[9][9] = static_cast<real_t>(points.size());

    for (uint_t m = 1; m < 10; m++)
    {
        for (uint_t n = 0; n < m; n++)
        {
            matA[m][n] = matA[n][m];
        }
    }

    real_t matB[10][10];
    eigenCompute(matA, vecA, matB, 10);

    uint_t j = 0;
    for (uint_t i = 1; i < 10; i++)
    {
        if (vecA[i] < vecA[j])
        {
            j = i;
        }
    }
    Matrix3x3 A;
    A[0][0] = matB[0][j];
    A[0][1] = A[1][0] = matB[1][j];
    A[0][2] = A[2][0] = matB[2][j];
    A[1][1] = matB[3][j];
    A[1][2] = A[2][1] = matB[4][j];
    A[2][2] = matB[5][j];

    real_t det = A.determinant();
    if (det < 0)
    {
        A *= -1;
        matB[6][j] = -matB[6][j];
        matB[7][j] = -matB[7][j];
        matB[8][j] = -matB[8][j];
        matB[9][j] = -matB[9][j];
        det = -det;
    }

    Matrix3x3 invA = A.inverseSymmetric();

    bias.zero();
    for (uint_t m = 0; m < 3; m++)
    {
        bias.x += invA[0][m] * matB[m + 6][j];
        bias.y += invA[1][m] * matB[m + 6][j];
        bias.z += invA[2][m] * matB[m + 6][j];
    }
    bias *= -0.5;

    real_t fieldStrength = Math::abs(A[0][0] * bias.x * bias.x +
        2.0 * A[0][1] * bias.x * bias.y +
        2.0 * A[0][2] * bias.x * bias.z +
        A[1][1] * bias.y * bias.y +
        2.0 * A[1][2] * bias.y * bias.z +
        A[2][2] * bias.z * bias.z - matB[9][j]);

    fitError = Math::sqrt(Math::abs(vecA[j]) / points.size()) / fieldStrength;
    fieldStrength = Math::sqrt(fieldStrength) * Math::pow(det, -1.0 / 6.0);
    A *= Math::pow(det, -1.0 / 3.0);

    for (uint_t i = 0; i < 3; i++)
    {
        for (j = 0; j < 3; j++)
        {
            matA[i][j] = A[i][j];
        }
    }
    eigenCompute(matA, vecA, matB, 3);

    for (j = 0; j < 3; j++)
    {
        real_t tmp = Math::sqrt(Math::sqrt(Math::abs(vecA[j])));
        for (uint_t i = 0; i < 3; i++)
        {
            matB[i][j] *= tmp;
        }
    }

    for (uint_t i = 0; i < 3; i++)
    {
        for (j = i; j < 3; j++)
        {
            transform[i][j] = 0.0;
            for (uint_t k = 0; k < 3; k++)
            {
                transform[i][j] += matB[i][k] * matB[j][k];
            }
            transform[j][i] = transform[i][j];
        }
    }
    return fitError;
}
//--------------------------------------------------------------------------------------------------
void eigenCompute(real_t A[][10], real_t eigVal[], real_t eigVec[][10], int_t n)
{
    for (int_t r = 0; r < n; r++)
    {
        for (int_t c = 0; c < n; c++)
        {
            eigVec[r][c] = 0.0;
        }

        eigVec[r][r] = 1.0;
        eigVal[r] = A[r][r];
    }

    uint_t maxLoops = 15;
    real_t residue = 0.0;
    do
    {
        for (int_t r = 0; r < n - 1; r++)
        {
            for (int_t c = r + 1; c < n; c++)
            {
                residue += Math::abs(A[r][c]);
            }
        }

        if (residue > 0.0)
        {
            for (int_t r = 0; r < n - 1; r++)
            {
                for (int_t c = r + 1; c < n; c++)
                {
                    if (Math::abs(A[r][c]) > 0.0)
                    {
                        real_t cot2phi = 0.5 * (eigVal[c] - eigVal[r]) / (A[r][c]);
                        real_t tanphi = 1.0 / (Math::abs(cot2phi) + Math::sqrt(1.0 + cot2phi * cot2phi));
                        if (cot2phi < 0.0)
                        {
                            tanphi = -tanphi;
                        }

                        real_t cosphi = 1.0 / Math::sqrt(1.0 + tanphi * tanphi);
                        real_t sinphi = tanphi * cosphi;
                        real_t tanhalfphi = sinphi / (1.0 + cosphi);
                        real_t tmp = tanphi * A[r][c];
                        eigVal[r] -= tmp;
                        eigVal[c] += tmp;
                        A[r][c] = 0.0;

                        for (int_t j = 0; j < n; j++)
                        {
                            tmp = eigVec[j][r];
                            eigVec[j][r] = tmp - sinphi * (eigVec[j][c] + tanhalfphi * tmp);
                            eigVec[j][c] = eigVec[j][c] + sinphi * (tmp - tanhalfphi * eigVec[j][c]);
                        }

                        for (int_t j = 0; j <= r - 1; j++)
                        {
                            tmp = A[j][r];
                            A[j][r] = tmp - sinphi * (A[j][c] + tanhalfphi * tmp);
                            A[j][c] = A[j][c] + sinphi * (tmp - tanhalfphi * A[j][c]);
                        }
                        for (int_t j = r + 1; j <= c - 1; j++)
                        {
                            tmp = A[r][j];
                            A[r][j] = tmp - sinphi * (A[j][c] + tanhalfphi * tmp);
                            A[j][c] = A[j][c] + sinphi * (tmp - tanhalfphi * A[j][c]);
                        }
                        for (int_t j = c + 1; j < n; j++)
                        {
                            tmp = A[r][j];
                            A[r][j] = tmp - sinphi * (A[c][j] + tanhalfphi * tmp);
                            A[c][j] = A[c][j] + sinphi * (tmp - tanhalfphi * A[c][j]);
                        }
                    }
                }
            }
        }
    } while ((residue > 0) && maxLoops--);

    return;
}
//--------------------------------------------------------------------------------------------------