#ifndef AHRS_H_
#define AHRS_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sigSlot.h"
#include "maths/vector.h"
#include "maths/matrix.h"
#include "maths/quaternion.h"

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    class Ahrs
    {
    public:
        const uint_t& deviceId = m_deviceId;                    ///< The device ID of the AHRS. This is the same as the device ID that owns the AHRS.

        /**
        * @brief A subscribable event for AHRS data.
        * @param ahrs Ahrs& The AHRS that raised the event.
        * @param timestampUs uint64_t The timestamp of the AHRS data in microseconds.
        * @param quaternion Quaternion& The orientation data expressed as a quaternion.
        * @param magneticHeading real_t A tilt compensated magnetic heading in radians from the magnetometer.
        * @param turnCount real_t The number of turns about the set axis.
        */
        Signal<Ahrs&, uint64_t, const Math::Quaternion&, real_t, real_t> onData;

        template<typename T>
        Ahrs(uint_t deviceId, T* inst, void (T::* setHead)(const real_t*), void (T::* clrTurns)())
            : m_deviceId(deviceId),
            m_setHeading([=](const real_t* heading) {(inst->*setHead)(heading); }),
            m_clearTurnsCount([=]() {(inst->*clrTurns)(); })
        {
        }
        ~Ahrs() {}

        /**
        * @brief Set the heading of the AHRS.
        * @param headingRad The heading in radians.
        */
        void setHeading(real_t headingRad);

        /**
        * @brief Set the heading of the AHRS to the magnetic heading.
        */
        void setHeadingToMag();

        /**
        * @brief Clear the turn count of the AHRS.
        */
        void clearTurnsCount();

    private:
        uint_t m_deviceId;
        std::function<void(const real_t*)> m_setHeading;
        std::function<void()> m_clearTurnsCount;
    };

    class GyroSensor
    {
    public:
        const uint_t& deviceId = m_deviceId;            ///< The device ID of the gyro sensor. This is the same as the device ID that owns the gyro sensor.
        const uint_t& sensorNumber = m_sensor;          ///< The sensor number of the gyro sensor.
        const Math::Vector3& bias = m_bias;                   ///< The bias values for the gyro sensor.

        /**
        * @brief A subscribable event for gyro data.
        * @param sensor GyroSensor& The gyro sensor that raised the event.
        * @param vector Math::Vector3& The gyro sensor reading.
        */
        Signal<GyroSensor&, const Math::Vector3&> onData;

        /**
        * @brief A subscribable event for gyro calibration changes.
        * @param sensor GyroSensor& The gyro sensor that raised the event.
        * @param vector Math::Vector3& The new bias values.
        */
        Signal<GyroSensor&, const Math::Vector3&> onCalChange;

        template<typename T>
        GyroSensor(uint_t deviceId, uint_t sensor, T* inst, void (T::* func)(uint_t, const Math::Vector3*)) : m_deviceId(deviceId), m_sensor(sensor), m_sendCal([=](uint_t sensorNum, const Math::Vector3* v) {(inst->*func)(sensorNum, v); }) {}
        ~GyroSensor() {}

        /**
        * @brief Update the calibration values within this class.
        * @param bias The new bias values.
        */
        void updateCalValues(Math::Vector3& bias);

        /**
        * @brief Automatically determine and set the calibration values for the gyro sensor.
        * The sensor can be in any orintation when this is called, but it must be completely still.
        */
        void autoCal();

        /**
        * @brief Set the calibration values for the gyro sensor.
        * @param bias The new bias values.
        */
        void setCal(Math::Vector3& bias);

    private:
        uint_t m_deviceId;
        uint_t m_sensor;
        Math::Vector3 m_bias;
        std::function<void(uint_t, const Math::Vector3*)> m_sendCal;
    };

    class AccelSensor
    {
    public:
        const uint_t& deviceId = m_deviceId;                ///< The device ID of the accel sensor. This is the same as the device ID that owns the accel sensor.
        const uint_t& sensorNumber = m_sensor;              ///< The sensor number of the accel sensor.
        const Math::Vector3& bias = m_bias;                       ///< The bias values for the accel sensor.
        const Math::Matrix3x3& transform = m_transform;           ///< The transform matrix for the accel sensor.

        /**
        * @brief A subscribable event for accel data.
        * @param sensor AccelSensor& The accel sensor that raised the event.
        * @param vector Math::Vector3& The accel sensor reading.
        */
        Signal<AccelSensor&, const Math::Vector3&> onData;

        /**
        * @brief A subscribable event for accel calibration changes.
        * @param sensor AccelSensor& The accel sensor that raised the event.
        * @param vector Math::Vector3& The new bias values.
        * @param matrix Math::Matrix3x3& The new transform matrix.
        */
        Signal<AccelSensor&, const Math::Vector3&, const Math::Matrix3x3&> onCalChange;

        /**
        * @brief A subscribable event for accel calibration progress.
        * @param sensor AccelSensor& The accel sensor that raised the event.
        * @param vector Math::Vector3& The data that is being used to calibrate the axis.
        * @param count uint_t The number of the data points that is being used to calibrate the sensor.
		*/
        Signal<AccelSensor&, const Math::Vector3&, uint_t> onCalProgress;

        template<typename T>
        AccelSensor(uint_t deviceId, uint_t sensor, T* inst, void (T::* func)(uint_t, const Math::Vector3&, const Math::Matrix3x3&)) : m_deviceId(deviceId), m_sensor(sensor), m_sendCal([=](uint_t sensorNum, const Math::Vector3& v, const Math::Matrix3x3& m) {(inst->*func)(sensorNum, v, m); }) {}
        ~AccelSensor() {}

        /**
        * @brief Update the calibration values within this class.
        * @param bias The new bias values.
        * @param transform The new transform matrix.
        */
        void updateCalValues(const Math::Vector3& bias, const Math::Matrix3x3& transform);

        /**
        * @brief Set the calibration values for the accel sensor.
        * @param bias The new bias values.
        * @param transform The new transform matrix.
        */
        void setCal(const Math::Vector3& bias, const Math::Matrix3x3& transform);

        /**
        * @brief Start the calibration process for the accel sensor.
        * This starts the accelerometer calibration process. The sensor must be place in the 6 cardinal orientations
        * and kept still for the duration of an axis calibration (the time to aquire \p samplesPerAverage number of samples).
        * If any of the samples deviate from the average by more than \p maxVariationG, or the angle between the sample vector
        * and the cardinal axis is greater than \p degFromCardinal the calibration for the axis will reset.
        * @param samplesPerAverage The number of samples to average for each axis.
        * @param maxVariationG The maximum allowable variation in g between the number of samples that are averaged.
        * @param factoryCal True for factory calibration, false for user calibration.
        */
        void startCal(uint_t samplesPerAverage, real_t maxVariationG, bool_t factoryCal = false);

        /**
        * @brief Stop the calibration process for the accel sensor.
        * This stops the accelerometer calibration process. If \p cancel is false and data has been aquired for all
        * 6 cardinal axis then the calibration is saved to the device.
        * @param cancel If true the calibration process will be cancelled and the calibration values will not be updated.
        * @param[out] biasCorrection Math::Vector3* This can be nullptr. As calibrations are compounded this bias correction can be used to correct the aquired data.
        * @param[out] transformCorrection Math::Matrix3x3* This can be nullptr. As calibrations are compounded this transform correction can be used to correct the aquired data.
        * @return bool_t True if the calibration was successful.
        */
        bool_t stopCal(bool_t cancel, Math::Vector3* biasCorrection=nullptr, Math::Matrix3x3* transformCorrection=nullptr);

    private:
        uint_t m_deviceId;
        uint_t m_sensor;
        Math::Vector3 m_bias;
        Math::Matrix3x3 m_transform;
        void newData(AccelSensor&, const Math::Vector3& v);
        Slot<AccelSensor&, const Math::Vector3&> slotData{ this, & AccelSensor::newData };
        std::function<void(uint_t, const Math::Vector3&, const Math::Matrix3x3&)> m_sendCal;

        struct Cal
        {
            uint_t averageCount;
            real_t maxAllowableSqG;
            Math::Vector3 average;
            uint_t count;
            bool_t factoryCal;
            std::vector<Math::Vector3> calData;

            Cal() : averageCount(0), maxAllowableSqG(0), count(0), factoryCal(false) {}
            bool_t calculate(Math::Vector3& bias, Math::Matrix3x3& transform);
        } m_cal;
    };

    class MagSensor
    {
    public:
        const uint_t& deviceId = m_deviceId;                    ///< The device ID of the mag sensor. This is the same as the device ID that owns the mag sensor.
        const uint_t& sensorNumber = m_sensor;                  ///< The sensor number of the mag sensor.
        const Math::Vector3& bias = m_bias;                     ///< The bias values for the mag sensor.
        const Math::Matrix3x3& transform = m_transform;         ///< The transform matrix for the mag sensor.

        /**
        * @brief A subscribable event for mag data.
        * @param sensor MagSensor& The mag sensor that raised the event.
        * @param vector Math::Vector3& The mag sensor reading.
        */
        Signal<MagSensor&, const Math::Vector3&> onData;

        /**
        * @brief A subscribable event for mag calibration changes.
        * @param sensor MagSensor& The mag sensor that raised the event.
        * @param vector Math::Vector3& The new bias values.
        * @param matrix Math::Matrix3x3& The new transform matrix.
        */
        Signal<MagSensor&, const Math::Vector3&, const Math::Matrix3x3&> onCalChange;

        /**
        * @brief A subscribable event for mag calibration progress.
        * @param sensor MagSensor& The mag sensor that raised the event.
        * @param vector Math::Vector3& The data that will be used to calibrate the mag.
        * @param count uint_t The number of the data points that is being used to calibrate the sensor.
        */
        Signal<MagSensor&, const Math::Vector3&, uint_t> onCalProgress;

        template<typename T>
        MagSensor(uint_t deviceId, uint_t sensor, T* inst, void (T::* func)(uint_t, const Math::Vector3&, const Math::Matrix3x3&, bool_t), void(T::* loadCal)()) : 
            m_deviceId(deviceId),
            m_sensor(sensor),
            m_sendCal([=](uint_t sensorNum, const Math::Vector3& v, const Math::Matrix3x3& m, bool_t factoryCal) {(inst->*func)(sensorNum, v, m, factoryCal); }), 
            m_loadFactoryCal([=]() {(inst->*loadCal)(); }) {}
        ~MagSensor() {}

        /**
        * @brief Update the calibration values within this class.
        * @param bias The new bias values.
        * @param transform The new transform matrix.
        */
        void updateCalValues(const Math::Vector3& bias, const Math::Matrix3x3& transform);

        /**
        * @brief Set the calibration values for the mag sensor.
        * @param bias The new bias values.
        * @param transform The new transform matrix.
        * @param factoryCal True for factory calibration, false for user calibration.
        */
        void setCal(const Math::Vector3& bias, const Math::Matrix3x3& transform, bool_t factoryCal=false);


        /**
        * @brief Reset the calibration values for the mag sensor to the factory values.
        * This will reset the calibration values to the factory values.
        */
        void resetToFactoryCal();

        /**
        * @brief Start the calibration process for the mag sensor.
        * This starts the magnetometer calibration process. The sensor must be rotated in all directions and kept 
        * still at each orientation if passing in an accelerometer.
        * @param spreadUt The minimum spread in microteslas between mag data points.
        * @param cal2D True to only calibrate the x and y axis, false to calibrate all 3 axis.
        * @param accel The accelerometer to use for factory calibration. This can be nullptr.
        */
        void startCal(real_t spreadUt=10, bool_t cal2D=false, AccelSensor* accel=nullptr);

        /**
        * @brief Stop the calibration process for the mag sensor.
        * This stops the magnetometer calibration process. If \p cancel is false the aquired data will be used to calculate the calibration values.
        * @param cancel If true the calibration process will be cancelled and the calibration values will not be updated.
        * @param[out] biasCorrection Math::Vector3* This can be nullptr. As calibrations are compounded this bias correction can be used to correct the aquired data.
        * @param[out] transformCorrection Math::Matrix3x3* This can be nullptr. As calibrations are compounded this transform correction can be used to correct the aquired data.
        * @return bool_t True if the calibration was successful.
        */
        bool_t stopCal(bool_t cancel, Math::Vector3* biasCorrection=nullptr, Math::Matrix3x3* transformCorrection=nullptr);

    private:
        uint_t m_deviceId;
        uint_t m_sensor;
        Math::Vector3 m_bias;
        Math::Matrix3x3 m_transform;
        void newData(MagSensor&, const Math::Vector3& v);
        void newAccelData(AccelSensor& accel, const Math::Vector3& v);
        Slot<MagSensor&, const Math::Vector3&> slotData{ this, & MagSensor::newData };
        Slot<AccelSensor&, const Math::Vector3&> slotAccelData{ this, & MagSensor::newAccelData };
        std::function<void(uint_t, const Math::Vector3&, const Math::Matrix3x3&, bool_t)> m_sendCal;
        std::function<void(void)> m_loadFactoryCal;
        

        class Cal
        {
        public:
            AccelSensor* accel;
            Math::Vector3 accelAverage;
            uint_t accelCount;
            const uint_t accelAverageCount = 10;
            const real_t maxAllowableSqG = 0.01 * 0.01;
            real_t spreadUt2;
            bool_t cal2D;
            std::vector<Math::Vector3> magData;
            std::vector<Math::Vector3> accelData;
            Cal() : accel(nullptr), accelCount(0), spreadUt2(0) {}
            bool_t calculate(Math::Vector3& bias, Math::Matrix3x3& transform);
        } m_cal;
    };
}
//--------------------------------------------------------------------------------------------------
#endif
