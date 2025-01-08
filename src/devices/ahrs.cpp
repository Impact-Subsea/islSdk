//------------------------------------------ Includes ----------------------------------------------

#include "ahrs.h"
#include "maths/maths.h"
#include "maths/matrix.h"
#include "maths/vector.h"
#include "maths/eigen.h"
#include "platform/debug.h"

using namespace IslSdk;

bool_t ellipsoidFit13Ls(const std::vector<Math::Vector3>& mag, const std::vector<Math::Vector3>& accel, Math::Vector3& bias, Math::Matrix3x3& transform);
bool_t ellipsoidFit12Ls(const std::vector<Math::Vector3>& data, Math::Vector3& bias, Math::Matrix3x3& transform);
bool_t ellipsoidFit10(const std::vector<Math::Vector3>& magData, Math::Vector3& bias, Math::Matrix3x3& transform);
bool_t ellipseFit6(const std::vector<Math::Vector2>& magData, Math::Vector3& bias, Math::Matrix3x3& transform);
Math::Vector3 planeOfBestFit(const std::vector<Math::Vector3>& data);

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
void GyroSensor::updateCalValues(Math::Vector3& bias)
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
void GyroSensor::setCal(Math::Vector3& bias)
{
    updateCalValues(bias);
    m_sendCal(m_sensor, &bias);
}
//==================================================================================================
void AccelSensor::updateCalValues(const Math::Vector3& bias, const Math::Matrix3x3& transform)
{
    if (m_bias != bias || m_transform != transform)
    {
        m_bias = bias;
        m_transform = transform;
        onCalChange(*this, m_bias, m_transform);
    }
}
//--------------------------------------------------------------------------------------------------
void AccelSensor::setCal(const Math::Vector3& bias, const Math::Matrix3x3& transform)
{
    m_sendCal(m_sensor, bias, transform);
    updateCalValues(bias, transform);
}
//--------------------------------------------------------------------------------------------------
void AccelSensor::startCal(uint_t samplesPerAverage, real_t maxAllowableG, bool_t factoryCal)
{
    m_cal.averageCount = samplesPerAverage;
    m_cal.maxAllowableSqG = maxAllowableG * maxAllowableG;
    m_cal.average.zero();
    m_cal.count = m_cal.averageCount;
    m_cal.factoryCal = factoryCal;
    m_cal.calData.clear();
    onData.connect(slotData);
}
//--------------------------------------------------------------------------------------------------
bool_t AccelSensor::stopCal(bool_t cancel, Math::Vector3* biasCorrection, Math::Matrix3x3* transformCorrection)
{
    Math::Matrix3x3 transform;
    Math::Vector3 bias;
    bool_t ok = false;

    onData.disconnect(slotData);

    if (!cancel && m_cal.calculate(bias, transform))
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
    m_cal.calData.clear();
    return ok;
}
//--------------------------------------------------------------------------------------------------
bool_t AccelSensor::Cal::calculate(Math::Vector3& bias, Math::Matrix3x3& transform)
{
    if (factoryCal)
    {
        return ellipsoidFit12Ls(calData, bias, transform);
    }
    else if (calData.size() == 2)
    {
        Math::Vector3 sensorDown = calData[0].normalise();
        Math::Vector3 sensorFwd = calData[1].normalise();

        Math::Vector3 fwd;
        fwd.z = Math::sqrt(1.0 - sensorDown.z * sensorDown.z);
        real_t scale = Math::sqrt((1.0 - fwd.z * fwd.z) / (sensorFwd.x * sensorFwd.x + sensorFwd.y * sensorFwd.y));
        fwd.x = sensorFwd.x * scale;
        fwd.y = sensorFwd.y * scale;
        transform = Math::Matrix3x3::fromDownAndForward(sensorDown, fwd);
        bias.zero();
        return true;
    }
    return false;
}
//--------------------------------------------------------------------------------------------------
void AccelSensor::newData(AccelSensor& accel, const Math::Vector3& v)
{
    if (m_cal.count)
    {
        if ((m_cal.average - v).magnitudeSq() < m_cal.maxAllowableSqG)
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
            int_t idx = -1;
            Math::Vector3 averageNorm = m_cal.average.normalise();
            m_cal.count = m_cal.averageCount;
            Math::Vector3 value = m_cal.average;
            m_cal.average.zero();

            if (m_cal.factoryCal)
            {
                for (const Math::Vector3& calValue : m_cal.calData)
				{
					if (averageNorm.dot(calValue.normalise()) >= Math::cos(Math::degToRad(15)))
					{
						return;
					}
				}   

                m_cal.calData.push_back(value);
            }
            else
            {
                if (m_cal.calData.empty())
                {
                    m_cal.calData.push_back(value);
                }
                else if (m_cal.calData.size() == 1)
                {
                    real_t dot = averageNorm.dot(m_cal.calData[0].normalise());

                    if (Math::abs(dot) <= Math::cos(Math::degToRad(85)))
                    {
                        m_cal.calData.push_back(value);
                    }
                }
                else
                {
                    return;
                }
            }

            onCalProgress(*this, value, m_cal.calData.size());
        }
    }
}
//==================================================================================================
void MagSensor::updateCalValues(const Math::Vector3& bias, const Math::Matrix3x3& transform)
{
    if (m_bias != bias || m_transform != transform)
    {
        m_bias = bias;
        m_transform = transform;
        onCalChange(*this, m_bias, m_transform);
    }
}
//--------------------------------------------------------------------------------------------------
void MagSensor::setCal(const Math::Vector3& bias, const Math::Matrix3x3& transform, bool_t factoryCal)
{
    m_sendCal(m_sensor, bias, transform, factoryCal);
    if (!factoryCal)
    {
        updateCalValues(bias, transform);
    }
}
//--------------------------------------------------------------------------------------------------
void MagSensor::resetToFactoryCal()
{
    m_loadFactoryCal();
}
//--------------------------------------------------------------------------------------------------
void MagSensor::startCal(real_t spreadUt, bool_t cal2D, AccelSensor* accel)
{
    m_cal.accel = accel;
    m_cal.spreadUt2 = spreadUt * spreadUt;
    m_cal.cal2D = cal2D;
    m_cal.accelAverage.zero();
    m_cal.accelCount = 0;
    m_cal.magData.clear();
    m_cal.accelData.clear();
    onData.connect(slotData);

    if (m_cal.accel)
    {
        m_cal.accel->onData.connect(slotAccelData);
        m_cal.accelCount = m_cal.accelAverageCount;
    }
}
//--------------------------------------------------------------------------------------------------
bool_t MagSensor::stopCal(bool_t cancel, Math::Vector3* biasCorrection, Math::Matrix3x3* transformCorrection)
{
    Math::Matrix3x3 transform;
    Math::Vector3 bias;
    bool_t ok = false;

    onData.disconnect(slotData);
    if (m_cal.accel)
    {
        m_cal.accel->onData.disconnect(slotAccelData);
    }

    if (!cancel)
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
            setCal(bias, transform, m_cal.accel != nullptr);
            ok = true;
        }
    }

    m_cal.accel = nullptr;

    return ok;
}
//--------------------------------------------------------------------------------------------------
bool_t MagSensor::Cal::calculate(Math::Vector3& bias, Math::Matrix3x3& transform)
{
    bool_t ok = false;

    if (accel)
    {
        ok = ellipsoidFit13Ls(magData, accelData, bias, transform);
    }
    else
    {
        if (cal2D)
        {
            Math::Vector3 norm = planeOfBestFit(magData);

            Math::Vector3 axis = norm.cross(Math::Vector3(0, 0, 1));
            Math::Matrix3x3 rotation = Math::Matrix3x3::rodrigues(axis, norm.dot(Math::Vector3(0, 0, 1)));
;
            std::vector<Math::Vector2> magData2D(magData.size());

            for (uint_t i = 0; i < magData.size(); i++)
            {
                Math::Vector3 magDataRotated = rotation * magData[i];
                magData2D[i].x = magDataRotated.x;
                magData2D[i].y = magDataRotated.y;
            }

            ok = ellipseFit6(magData2D, bias, transform);
            transform = rotation.transpose() * transform * rotation;
            bias = rotation.transpose() * bias;
        }
        else
        {
            ok = ellipsoidFit10(magData, bias, transform);
        }
    }
    return ok;
}
//--------------------------------------------------------------------------------------------------
void MagSensor::newData(MagSensor& mag, const Math::Vector3& magVector)
{
    Math::Vector3 accel;

    if (m_cal.accel)
    {
        if (m_cal.accelCount)
        {
            return;
        }
        accel = m_cal.accelAverage;
        m_cal.accelAverage.zero();
        m_cal.accelCount = m_cal.accelAverageCount;
    }

    for (uint_t i = 0; i < m_cal.magData.size(); i++)
    {
        if ((magVector - m_cal.magData[i]).magnitudeSq() < m_cal.spreadUt2)
        {
            return;
        }
    }

    if (m_cal.accel)
    {
        m_cal.accelData.push_back(accel);
    }
    m_cal.magData.push_back(magVector);
    onCalProgress(*this, magVector, m_cal.magData.size());
}
//--------------------------------------------------------------------------------------------------
void MagSensor::newAccelData(AccelSensor& accel, const Math::Vector3& v)
{
    if (m_cal.accelCount)
    {
        if ((m_cal.accelAverage - v).magnitudeSq() < m_cal.maxAllowableSqG)
        {
            m_cal.accelAverage += v;
            m_cal.accelAverage *= 0.5;
        }
        else
        {
            m_cal.accelAverage = v;
            m_cal.accelCount = m_cal.accelAverageCount;
        }
        m_cal.accelCount--;
    }
}
//==================================================================================================
/*
* This algorithm is based on the paper "Calibration of tri-axial magnetometer using vector observations and inner products" by Xiang Li
* https://www.researchgate.net/publication/328183770_Calibration_of_tri-axial_magnetometer_using_vector_observations_and_inner_products
* It use the marquardt-levenberg algorithm to damp Gauss-Newton least squares method to minimize the cost function which is the sum of the
* residuals of the magnetometer and accelerometer data. It calcualtes the hard iron bias vector and the soft iron matrix and as well
* as the rotation matrix to align the magnetometer axis with the accelerometer axis. The soft iron matrix and rotation matrix
* are combined into a single transformation matrix.
*/
bool_t ellipsoidFit13Ls(const std::vector<Math::Vector3>& mag, const std::vector<Math::Vector3>& accel, Math::Vector3& bias, Math::Matrix3x3& transform)
{
    const uint_t N = 13;
    if (mag.size() <= N || mag.size() != accel.size())
    {
        return false;
    }
    const real_t epsilon = 0.000001;
    uint_t maxIterations = 20;
    real_t lambda = 10;
    Math::Vector<N> beta(1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0);
    real_t averageH = 0;

    for (const Math::Vector3& v : mag)
    {
        averageH += v.magnitude();
    }
    averageH /= mag.size();

    while (maxIterations--)
    {
        Math::Matrix<N, N> jTj;
        Math::Vector<N> gradCost;
        real_t cost = 0;

        for (uint_t idx = 0; idx < mag.size(); idx++)          // Create jacobian matrix j and compute jTj = j' * j along with gradCost = j' * residualVector
        {
            const Math::Vector3& m = mag[idx];
            const Math::Vector3& a = accel[idx];

            real_t bvx = m.x - beta[9];
            real_t bvy = m.y - beta[10];
            real_t bvz = m.z - beta[11];
            real_t hx = beta[0] * bvx + beta[1] * bvy + beta[2] * bvz;
            real_t hy = beta[3] * bvx + beta[4] * bvy + beta[5] * bvz;
            real_t hz = beta[6] * bvx + beta[7] * bvy + beta[8] * bvz;
            real_t residualMag = (hx * hx + hy * hy + hz * hz) - averageH * averageH;
            real_t residualAccel = (a.x * hx + a.y * hy + a.z * hz) - averageH * beta[12];
            cost += residualMag * residualMag + residualAccel * residualAccel;

            real_t grad[N * 2];
            grad[0] = 2 * hx * bvx;
            grad[1] = 2 * hx * bvy;
            grad[2] = 2 * hx * bvz;
            grad[3] = 2 * hy * bvx;
            grad[4] = 2 * hy * bvy;
            grad[5] = 2 * hy * bvz;
            grad[6] = 2 * hz * bvx;
            grad[7] = 2 * hz * bvy;
            grad[8] = 2 * hz * bvz;
            grad[9] = -2 * (beta[0] * hx + beta[3] * hy + beta[6] * hz);
            grad[10] = -2 * (beta[1] * hx + beta[4] * hy + beta[7] * hz);
            grad[11] = -2 * (beta[2] * hx + beta[5] * hy + beta[8] * hz);
            grad[12] = 0;
            grad[N + 0] = a.x * bvx;
            grad[N + 1] = a.x * bvy;
            grad[N + 2] = a.x * bvz;
            grad[N + 3] = a.y * bvx;
            grad[N + 4] = a.y * bvy;
            grad[N + 5] = a.y * bvz;
            grad[N + 6] = a.z * bvx;
            grad[N + 7] = a.z * bvy;
            grad[N + 8] = a.z * bvz;
            grad[N + 9] = -(beta[0] * a.x + beta[3] * a.y + beta[6] * a.z);
            grad[N + 10] = -(beta[1] * a.x + beta[4] * a.y + beta[7] * a.z);
            grad[N + 11] = -(beta[2] * a.x + beta[5] * a.y + beta[8] * a.z);
            grad[N + 12] = -averageH;

            for (uint_t i = 0; i < N; i++)
            {
                gradCost[i] += grad[i] * residualMag + grad[i + N] * residualAccel;     // gradCost = j' * residualVector

                for (uint_t j = i; j < N; j++)
                {
                    jTj[i][j] += grad[i] * grad[j] + grad[i + N] * grad[j + N];
                }
            }
        }
  
        for (uint_t i = 1; i < N; i++)              // Reflect the upper triangle to the lower triangle as jTj is symmetric
        {
            for (uint_t j = 0; j < i; j++)
            {
                jTj[i][j] = jTj[j][i];
            }
        }

        for (uint_t i = 0; i < N; i++)              // hessian = jacobian transpose * jacobian + lambdaIdentity * lambda
        {
            jTj[i][i] += lambda;
        }

        beta -= jTj.inverse() * gradCost;           // beta = beta - hessian inverse * gradCost

        real_t newCost = 0;
        for (uint_t idx = 0; idx < mag.size(); idx++)
        {
            const Math::Vector3& m = mag[idx];
            const Math::Vector3& a = accel[idx];

            real_t bvx = m.x - beta[9];
            real_t bvy = m.y - beta[10];
            real_t bvz = m.z - beta[11];
            real_t hx = beta[0] * bvx + beta[1] * bvy + beta[2] * bvz;
            real_t hy = beta[3] * bvx + beta[4] * bvy + beta[5] * bvz;
            real_t hz = beta[6] * bvx + beta[7] * bvy + beta[8] * bvz;
            real_t residualMag = (hx * hx + hy * hy + hz * hz) - averageH * averageH;
            real_t residualAccel = (a.x * hx + a.y * hy + a.z * hz) - averageH * beta[12];
            newCost += residualMag * residualMag + residualAccel * residualAccel;
        }

        if (Math::abs(cost - newCost) < epsilon)
        {
            transform[0][0] = beta[0];
            transform[0][1] = beta[1];
            transform[0][2] = beta[2];
            transform[1][0] = beta[3];
            transform[1][1] = beta[4];
            transform[1][2] = beta[5];
            transform[2][0] = beta[6];
            transform[2][1] = beta[7];
            transform[2][2] = beta[8];
            bias.x = beta[9];
            bias.y = beta[10];
            bias.z = beta[11];
            //real_t magInclination = Math::radToDeg(Math::pi / 2.0 - Math::acos(beta[12]));
            return true;
        }

        lambda *= newCost < cost ? 0.1 : 10.0;
    }
    return false;
}
//--------------------------------------------------------------------------------------------------
/*
* This algorithm uses the marquardt-levenberg algorithm to damp Gauss-Newton least squares method to minimize the cost function 
* which is the sum of the residuals of the magnetometer data. It calcualtes the hard iron bias vector and the 9 values of 
* the soft iron matrix
*/
bool_t ellipsoidFit12Ls(const std::vector<Math::Vector3>& data, Math::Vector3& bias, Math::Matrix3x3& transform)
{
    const uint_t N = 12;
    if (data.size() <= N)
    {
        return false;
    }

    const real_t epsilon = 0.0001;
    uint_t maxIterations = 20;
    real_t lambda = 10;
    Math::Vector<N> beta(1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0);
    
    while (maxIterations--)
    {
        Math::Matrix<N, N> jTj;
        Math::Vector<N> gradCost;
        real_t cost = 0;
        
        for (const Math::Vector3& v : data)          // Create jacobian matrix j and compute jTj = j' * j along with j' * residualVector
        {
            real_t residualX = beta[0] * v.x + beta[1] * v.y + beta[2] * v.z + beta[9];
            real_t residualY = beta[3] * v.x + beta[4] * v.y + beta[5] * v.z + beta[10];
            real_t residualZ = beta[6] * v.x + beta[7] * v.y + beta[8] * v.z + beta[11];
            real_t residual = (residualX * residualX + residualY * residualY + residualZ * residualZ) - 1;
            cost += residual * residual;

            real_t jacobian[N];
            jacobian[0] = 2.0 * residualX * v.x;
            jacobian[1] = 2.0 * residualX * v.y;
            jacobian[2] = 2.0 * residualX * v.z;
            jacobian[3] = 2.0 * residualY * v.x;
            jacobian[4] = 2.0 * residualY * v.y;
            jacobian[5] = 2.0 * residualY * v.z;
            jacobian[6] = 2.0 * residualZ * v.x;
            jacobian[7] = 2.0 * residualZ * v.y;
            jacobian[8] = 2.0 * residualZ * v.z;
            jacobian[9] = 2.0 * residualX;
            jacobian[10] = 2.0 * residualY;
            jacobian[11] = 2.0 * residualZ;

            for (uint_t i = 0; i < N; i++)
            {
                gradCost[i] += jacobian[i] * residual;      //gradCost = jacobian transpose * residual vector;

                for (uint_t j = i; j < N; j++)
                {
                    jTj[i][j] += jacobian[i] * jacobian[j];
                }
            }
        }

        for (uint_t i = 1; i < N; i++)              // Reflect the upper triangle to the lower triangle as jTj is symmetric
        {
            for (uint_t j = 0; j < i; j++)
            {
                jTj[i][j] = jTj[j][i];
            }
        }

        for (uint_t i = 0; i < N; i++)              // hessian = jacobian transpose * jacobian + lamdbaIdentity * lamdba
        {
            jTj[i][i] += lambda;
        }

        beta -= jTj.inverse() * gradCost;           // beta = beta - hessian inverse * gradCost

        real_t newCost = 0;
        for (const Math::Vector3& v : data)
        {
            real_t residualX = beta[0] * v.x + beta[1] * v.y + beta[2] * v.z + beta[9];
            real_t residualY = beta[3] * v.x + beta[4] * v.y + beta[5] * v.z + beta[10];
            real_t residualZ = beta[6] * v.x + beta[7] * v.y + beta[8] * v.z + beta[11];
            real_t residual = (residualX * residualX + residualY * residualY + residualZ * residualZ) - 1.0;
            newCost += residual * residual;
        }

        if (Math::abs(cost - newCost) < epsilon)
        {
            transform[0][0] = beta[0];
            transform[0][1] = beta[1];
            transform[0][2] = beta[2];
            transform[1][0] = beta[3];
            transform[1][1] = beta[4];
            transform[1][2] = beta[5];
            transform[2][0] = beta[6];
            transform[2][1] = beta[7];
            transform[2][2] = beta[8];
            bias.x = -beta[9];
            bias.y = -beta[10];
            bias.z = -beta[11];

            return true;
        }

        lambda *= newCost < cost ? 0.1 : 10.0;
    }
	return false;
}
//--------------------------------------------------------------------------------------------------
/*
* This algorithm uses quadric surface fitting to fit an ellipsoid to the magnetometer data. It calculates
* the hard iron bias vector and soft iron matrix which is always symmetric. The code is a port from NXP / Freescale
* https://github.com/memsindustrygroup/Open-Source-Sensor-Fusion/
*/
bool_t ellipsoidFit10(const std::vector<Math::Vector3>& magData, Math::Vector3& bias, Math::Matrix3x3& transform)
{
    const uint_t N = 10;
    if (magData.size() <= N)
    {
        return false;
    }
    Math::Matrix<N, N> dTd;

    for (const Math::Vector3& v : magData)          // Create design matrix d and compute dTd = d' * d
    {
        real_t vecA[N];
        vecA[0] = v.x * v.x;
        vecA[1] = 2.0 * v.x * v.y;
        vecA[2] = 2.0 * v.x * v.z;
        vecA[3] = v.y * v.y;
        vecA[4] = 2.0 * v.y * v.z;
        vecA[5] = v.z * v.z;
        vecA[6] = v.x;
        vecA[7] = v.y;
        vecA[8] = v.z;
        vecA[9] = 1;

        for (uint_t i = 0; i < N; i++)
        {
            for (uint_t j = i; j < N; j++)
            {
                dTd[i][j] += vecA[i] * vecA[j];
            }
        }
    }

    for (uint_t i = 0; i < N; i++)                  // Reflect the upper triangle to the lower triangle as dTd is symmetric
    {
        for (uint_t j = 0; j < i; j++)
        {
            dTd[i][j] = dTd[j][i];
        }
    }

    Math::Matrix<N, N> eigenVec;
    Math::Vector<N> eigenVal;
    Math::Eigen::computeSymmetric(dTd, eigenVal, eigenVec);

    uint_t idx = 0;
    for (uint_t i = 1; i < N; i++)
    {
        if (eigenVal[i] < eigenVal[idx])
        {
            idx = i;
        }
    }
    Math::Matrix3x3 A;
    A[0][0] = eigenVec[0][idx];
    A[0][1] = A[1][0] = eigenVec[1][idx];
    A[0][2] = A[2][0] = eigenVec[2][idx];
    A[1][1] = eigenVec[3][idx];
    A[1][2] = A[2][1] = eigenVec[4][idx];
    A[2][2] = eigenVec[5][idx];

    real_t det = A.determinant();
    if (det < 0)
    {
        A *= -1;
        eigenVec[6][idx] = -eigenVec[6][idx];
        eigenVec[7][idx] = -eigenVec[7][idx];
        eigenVec[8][idx] = -eigenVec[8][idx];
        eigenVec[9][idx] = -eigenVec[9][idx];
        det = -det;
    }

    Math::Matrix3x3 invA = A.inverseSymmetric();

    bias.zero();
    for (uint_t m = 0; m < 3; m++)
    {
        bias.x += invA[0][m] * eigenVec[m + 6][idx];
        bias.y += invA[1][m] * eigenVec[m + 6][idx];
        bias.z += invA[2][m] * eigenVec[m + 6][idx];
    }
    bias *= -0.5;

    real_t fieldStrength = Math::abs(A[0][0] * bias.x * bias.x + 2.0 * A[0][1] * bias.x * bias.y + 2.0 * A[0][2] * bias.x * bias.z +
        A[1][1] * bias.y * bias.y + 2.0 * A[1][2] * bias.y * bias.z + A[2][2] * bias.z * bias.z - eigenVec[9][idx]);

    real_t fitError = Math::sqrt(Math::abs(eigenVal[idx]) / magData.size()) / fieldStrength;
    fieldStrength = Math::sqrt(fieldStrength) * Math::pow(det, -1.0 / 6.0);
    A *= Math::pow(det, -1.0 / 3.0);

    Math::Matrix<3, 3> eigenVec3;
    Math::Vector<3> eigenVal3;
    Math::Eigen::computeSymmetric(A, eigenVal3, eigenVec3);

    for (uint_t j = 0; j < 3; j++)
    {
        real_t tmp = Math::sqrt(Math::sqrt(Math::abs(eigenVal3[j])));
        for (uint_t i = 0; i < 3; i++)
        {
            eigenVec3[i][j] *= tmp;
        }
    }

    for (uint_t i = 0; i < 3; i++)
    {
        for (uint_t j = i; j < 3; j++)
        {
            transform[i][j] = 0.0;
            for (uint_t k = 0; k < 3; k++)
            {
                transform[i][j] += eigenVec3[i][k] * eigenVec3[j][k];
            }
            transform[j][i] = transform[i][j];
        }
    }
    return true;
}
//--------------------------------------------------------------------------------------------------
//A 2D version of the ellipsoidFit10 function
bool_t ellipseFit6(const std::vector<Math::Vector2>& magData, Math::Vector3& bias, Math::Matrix3x3& transform)
{
    const uint_t N = 6;
    if (magData.size() <= N)
	{
		return false;
	}
    Math::Matrix<N, N> dTd;

    for (const Math::Vector2& v : magData)          // Create design matrix d and compute dTd = d' * d
    {
        real_t vecA[N];
        vecA[0] = v.x * v.x;
        vecA[1] = 2.0 * v.x * v.y;
        vecA[2] = v.y * v.y;
        vecA[3] = v.x;
        vecA[4] = v.y;
        vecA[5] = 1;

        for (uint_t i = 0; i < N; i++)
        {
            for (uint_t j = i; j < N; j++)
            {
                dTd[i][j] += vecA[i] * vecA[j];
            }
        }
    }

    for (uint_t i = 0; i < N; i++)                  // Reflect the upper triangle to the lower triangle as dTd is symmetric
    {
        for (uint_t j = 0; j < i; j++)
        {
            dTd[i][j] = dTd[j][i];
        }
    }

    Math::Matrix<N, N> eigenVec;
    Math::Vector<N> eigenVal;
    Math::Eigen::computeSymmetric(dTd, eigenVal, eigenVec);

    uint_t idx = 0;
    for (uint_t i = 1; i < N; i++)
    {
        if (eigenVal[i] < eigenVal[idx])
        {
            idx = i;
        }
    }
    
    Math::Matrix<2, 2> A(eigenVec[0][idx], eigenVec[1][idx], eigenVec[1][idx], eigenVec[2][idx]);

    real_t det = A.determinant();
    if (det < 0)
    {
        A *= -1;
        eigenVec[3][idx] = -eigenVec[3][idx];
        eigenVec[4][idx] = -eigenVec[4][idx];
        eigenVec[5][idx] = -eigenVec[5][idx];
        det = -det;
    }

    Math::Matrix invA = A.inverse();

    bias.zero();
    for (uint_t m = 0; m < 2; m++)
    {
        bias.x += invA[0][m] * eigenVec[m + 3][idx];
        bias.y += invA[1][m] * eigenVec[m + 3][idx];
    }
    bias *= -0.5;

    real_t fieldStrength = Math::abs(A[0][0] * bias.x * bias.x + 2.0 * A[0][1] * bias.x * bias.y + A[1][1] * bias.y * bias.y - eigenVec[5][idx]);
    real_t fitError = Math::sqrt(Math::abs(eigenVal[idx]) / magData.size()) / fieldStrength;
    fieldStrength = Math::sqrt(fieldStrength) * Math::pow(det, -1.0 / 4.0);
    A *= Math::pow(det, -1.0 / 2.0);

    Math::Matrix<2, 2> eigenVec3;
    Math::Vector<2> eigenVal3;
    Math::Eigen::computeSymmetric(A, eigenVal3, eigenVec3);

    for (uint_t j = 0; j < 2; j++)
    {
        real_t tmp = Math::sqrt(Math::sqrt(Math::abs(eigenVal3[j])));
        for (uint_t i = 0; i < 2; i++)
        {
            eigenVec3[i][j] *= tmp;
        }
    }

    transform[2][0] = transform[2][1] = transform[0][2] = transform[1][2] = 0.0;
    transform[2][2] = 1.0;

    for (uint_t i = 0; i < 2; i++)
    {
        for (uint_t j = i; j < 2; j++)
        {
            transform[i][j] = 0.0;
            for (uint_t k = 0; k < 2; k++)
            {
                transform[i][j] += eigenVec3[i][k] * eigenVec3[j][k];
            }
            transform[j][i] = transform[i][j];
        }
    }
    return true;
}
//--------------------------------------------------------------------------------------------------
Math::Vector3 planeOfBestFit(const std::vector<Math::Vector3>& data)
{
    const uint_t N = 4;
    Math::Matrix<N, N> dTd;
    
    for (const Math::Vector3& v : data)             // Create design matrix d and compute dTd = d' * d
    {
        real_t vec[N];
        vec[0] = v.x;
        vec[1] = v.y;
        vec[2] = v.z;
        vec[3] = 1;

        for (uint_t i = 0; i < N; i++)
        {
            for (uint_t j = i; j < N; j++)
            {
                dTd[i][j] += vec[i] * vec[j];
            }
        }
    }

    for (uint_t i = 1; i < N; i++)		  // Reflect the upper triangle to the lower triangle as dTd is symmetric
    {
        for (uint_t j = 0; j < i; j++)
        {
            dTd[i][j] = dTd[j][i];
        }
    }

    Math::Vector<4> eigVal;
    Math::Matrix<4, 4> eigVec;
    Math::Eigen::computeSymmetric(dTd, eigVal, eigVec);

    int_t idx = 0;
    for (uint_t i = 1; i < N; i++)
    {
        if (eigVal[i] < eigVal[idx])
        {
            idx = i;
        }
    }

    return Math::Vector3(eigVec[0][idx], eigVec[1][idx], eigVec[2][idx]).normalise();
}
//--------------------------------------------------------------------------------------------------