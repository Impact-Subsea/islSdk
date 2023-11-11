//------------------------------------------ Includes ----------------------------------------------

#include "ahrs.h"
#include "maths/maths.h"
#include "platform/debug.h"

using namespace IslSdk;

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

        transform.identity();
        transform.m[0][0] = (real_t)1.0 / ((calData[static_cast<uint_t>(Vector3::Axis::xPlus)].x - calData[static_cast<uint_t>(Vector3::Axis::xMinus)].x) * 0.5);
        transform.m[1][1] = (real_t)1.0 / ((calData[static_cast<uint_t>(Vector3::Axis::yPlus)].y - calData[static_cast<uint_t>(Vector3::Axis::yMinus)].y) * 0.5);
        transform.m[2][2] = (real_t)1.0 / ((calData[static_cast<uint_t>(Vector3::Axis::zPlus)].z - calData[static_cast<uint_t>(Vector3::Axis::zMinus)].z) * 0.5);

        Matrix3x3 m;
        m.m[0][0] = (calData[static_cast<uint_t>(Vector3::Axis::xPlus)].x - bias.x) * transform.m[0][0];
        m.m[0][1] = (calData[static_cast<uint_t>(Vector3::Axis::xPlus)].y - bias.y) * transform.m[1][1];
        m.m[0][2] = (calData[static_cast<uint_t>(Vector3::Axis::xPlus)].z - bias.z) * transform.m[2][2];

        m.m[1][0] = (calData[static_cast<uint_t>(Vector3::Axis::yPlus)].x - bias.x) * transform.m[0][0];
        m.m[1][1] = (calData[static_cast<uint_t>(Vector3::Axis::yPlus)].y - bias.y) * transform.m[1][1];
        m.m[1][2] = (calData[static_cast<uint_t>(Vector3::Axis::yPlus)].z - bias.z) * transform.m[2][2];

        m.m[2][0] = (calData[static_cast<uint_t>(Vector3::Axis::zPlus)].x - bias.x) * transform.m[0][0];
        m.m[2][1] = (calData[static_cast<uint_t>(Vector3::Axis::zPlus)].y - bias.y) * transform.m[1][1];
        m.m[2][2] = (calData[static_cast<uint_t>(Vector3::Axis::zPlus)].z - bias.z) * transform.m[2][2];

        Quaternion q(m);
        q = q.normalise().conjugate();

        transform = transform * q.toMatrix();
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

    if (!cancel && m_cal.points.size() > 10)
    {
        if (m_cal.calculate(54.0, bias, transform))
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
bool_t MagSensor::Cal::calculate(real_t radius, Vector3& bias, Matrix3x3& transform)
{
    Vector3 min = { 999999, 999999, 999999 }, max = { -999999, -999999, -999999 };

    for (const Vector3& v : points)
    {
        min.x = Math::min(min.x, v.x);
        min.y = Math::min(min.y, v.y);
        min.z = Math::min(min.z, v.z);

        max.x = Math::max(max.x, v.x);
        max.y = Math::max(max.y, v.y);
        max.z = Math::max(max.z, v.z);
    }

    bias = (min + max) * 0.5;

    transform.identity();
    transform.m[0][0] = radius / ((max.x - min.x) * 0.5);
    transform.m[1][1] = radius / ((max.y - min.y) * 0.5);
    transform.m[2][2] = radius / ((max.z - min.z) * 0.5);

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
