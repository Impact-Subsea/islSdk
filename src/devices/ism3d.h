#ifndef ISM3D_H_
#define ISM3D_H_

//------------------------------------------ Includes ----------------------------------------------

#include "device.h"
#include "maths/quaternion.h"
#include "maths/vector3.h"
#include "files/xmlFile.h"
#include "ahrs.h"
#include <string>

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    class Ism3d : public Device
    {
    public:
        struct Settings                                 /// Ism3d Settings information.
        {
            Device::UartMode uartMode;                  ///< Serial port mode.
            uint32_t baudrate;                          ///< Serial port baudrate. Limits are standard bauds between 300 and 115200.
            Device::Parity parity;                      ///< Serial parity.
            uint8_t dataBits;                           ///< Serial word length 5 to 8 bits.
            Device::StopBits stopBits;                  ///< Serial stop bits.
            uint8_t ahrsMode;                           ///< If bit zero is 1 use inertial mode. 0 is mag slave mode.
            Quaternion orientationOffset;               ///< Heading, pitch and roll offsets (or down and forward vectors) expressed as a quaternion.
            real_t headingOffsetRad;                    ///< Offset in radians to add to the heading. Typically use for magnetic declination.
            Vector3 turnsAbout;                         ///< A vector representing the axis which turn are measured about.
            bool_t turnsAboutEarthFrame;                ///< If true the "turnsAbout" vector is referenced to the earth frame. False is sensor frame.
            Device::CustomStr clrTurn;                  ///< The turns clearing string.
            Device::CustomStr setHeading2Mag;           ///< A string to set the heading to magnetometer heading.
            struct CustomStr
            {
                uint8_t strId;                          ///< Id of the string 0 = script
                bool_t intervalEnabled;                 ///< If true then autonomously aquire and output at the defined interval.
                uint32_t intervalMs;                    ///< Interval in milliseconds to autonomously output.
                Device::CustomStr interrogation;        ///< Custom interrogation string
            } strTrigger;

            static const uint_t size = 150;

            Settings();
            void defaults();
            bool_t check(std::vector<std::string>& errMsgs) const;
            uint_t serialise(uint8_t* buffer, uint_t size) const;
            uint_t deserialise(const uint8_t* buffer, uint_t size);
            bool_t load(const XmlElementPtr& node);
            void save(XmlElementPtr& node) const;
        };

        struct SensorRates
        {
            uint32_t ahrs;                              ///< Interval in milliseconds between AHRS data. Zero means no AHRS data.
            uint32_t gyro;                              ///< Interval in milliseconds between gyro data. Zero means no gyro data.
            uint32_t accel;                             ///< Interval in milliseconds between accelerometer data. Zero means no accelerometer data.
            uint32_t mag;                               ///< Interval in milliseconds between magnetometer data. Zero means no magnetometer data.
            SensorRates() : ahrs(0), gyro(0), accel(0), mag(0) {}
        };

        struct AhrsCal
        {
            Vector3 gyroBias;                           ///< Gyro bias corrections in degress per second.
            Vector3 accelBias;                          ///< Accel bias corrections in G.
            Vector3 magBias;                            ///< Mag bias corrections in uT.
            Matrix3x3 accelTransform;                   ///< Transformation matrix for accelerometer.
            Matrix3x3 magTransform;                     ///< Transformation matrix for magnetometer.
            Vector3 gyroBiasSec;                        ///< Backup Gyro bias corrections in degress per second.
            Vector3 accelBiasSec;                       ///< Backup Accel bias corrections in G.
            Matrix3x3 accelTransformSec;                ///< Backup Accel Transformation matrix for accelerometer.
        };

        Ahrs ahrs{ Device::id, this, &Ism3d::setHeading, &Ism3d::clearTurnsCount };     ///< Class to manage AHRS data.
        GyroSensor gyro{ Device::id, 0, this, &Ism3d::setGyroCal };                     ///< Class to manage gyro data.
        GyroSensor gyroSec{ Device::id, 1, this, &Ism3d::setGyroCal };                  ///< Class to manage backup gyro data.
        AccelSensor accel{ Device::id, 0, this, &Ism3d::setAccelCal };                  ///< Class to manage accelerometer data.
        AccelSensor accelSec{ Device::id, 1, this, &Ism3d::setAccelCal };               ///< Class to manage backup accelerometer data.
        MagSensor mag{ Device::id, 0, this, &Ism3d::setMagCal };                        ///< Class to manage magnetometer data.

        /**
        * @brief A subscribable event for knowing when the scriptVars, onPing and onAhrs data members are valid.
        * Subscribers to this signal will be called when scriptVars, onPing and onAhrs data has been received.
        * @param device Isa500& The device that triggered the event.
        */
        Signal<Ism3d&> onScriptDataReceived;

        /**
        * @brief A subscribable event for knowing when the settings have been updated.
        * Subscribers to this signal will be called when the settings have been updated.
        * @param device Isa500& The device that triggered the event.
        * @param success bool_t True if the settings were appiled to the device.
        */
        Signal<Ism3d&, bool_t> onSettingsUpdated;

        Ism3d(const Device::Info& info);
        ~Ism3d();

        /**
        * @brief Sets the data rates for the device.
        * This function is used to configure the sampling rates for the various sensors used in the system..
        * @param sensors A reference to the 'SensorRates' object containing the desired sampling rates for the sensors.
        */
        void setSensorRates(const SensorRates& sensors);

        /**
        * @brief Sets the device settings and optionally saves them.
        * @param settings The settings to be applied to the device.
        * @param save A boolean indicating whether to save the settings.
        * @return The boolean value indicating the success of the operation.
        * @note Device::onError event will be called if the settings are invalid.
        * @warning The EEPROM in this device has an endurance of 100000 write cycles. Writing to the EEPROM too often will
        * reduce the life of the device. Consider setting save to false if updating the settings frequently.
        */
        bool_t setSettings(const Settings& settings, bool_t save);

        /**
        * @brief Sets the AHRS (Attitude and Heading Reference System) script with the provided name and code.
        * The script is a custom impact subsea language that is executed on the device and is
        * used to format and build strings to be outputted when used without seaview or the SDK.
        * @param name The name of the AHRS script.
        * @param code The code for the AHRS script.
        * @return The boolean value indicating the success of the operation.
        */
        bool_t setAhrsScript(const std::string& name, const std::string& code);

        /**
        * @brief Retrieves the scripts.
        * @return The boolean value indicating the success of the operation. If false the scripts are not valid until an ::onScriptDataReceived event occurs.
        */
        bool_t getScripts();

        /**
        * @brief Starts logging for the device.
        */
        void startLogging() override;

        /**
        * @brief Saves the configuration with the provided file name.
        * @param fileName The name of the file to save the configuration.
        * @return The boolean value indicating the success of the operation.
        */
        bool_t saveConfig(const std::string& fileName) override;

        /**
        * @brief Loads the configuration from the provided file name.
        * Any of the pointers can be null if the corresponding data is not required.
        * @param fileName The name of the file from which to load the configuration.
        * @param info Pointer to the Device::Info object to load.
        * @param settings Pointer to the Settings object to load.
        * @param script0 Pointer to the first DeviceScript object to load.
        * @param cal Pointer to the AhrsCal object to load.
        * @return The boolean value indicating the success of the operation.
        */
        static bool_t loadConfig(const std::string& fileName, Device::Info* info, Settings* settings, DeviceScript* script0, AhrsCal* cal);

    public:
        const Settings& settings = m_settings;                                      ///< The current settings.
        const SensorRates& sensorsRates = m_requestedRates;                         ///< The sensor rates.
        const std::vector<std::string>& hardCodedAhrsOutputStrings = m_hardCodedAhrsOutputStrings;
        const ScriptVars& scriptVars = m_scriptVars;                                ///< The script variables.
        const DeviceScript& onAhrs = m_onAhrs;                                      ///< The AHRS script.

    private:
        enum class Commands
        {
            GetSensorData = 10,
            SetSensorInterval,
            GetSettings,
            SetSettings,
            GetAhrsCal,
            SetGyroCal,
            SetAccelCal,
            SetMagCal,
            SetHeading,
            ClearTurnsCount,
            GetStringNames,
            GetScriptVars,
            GetScript,
            SetScript,
        };

        Settings m_settings;
        std::unique_ptr<Settings> m_pendingSettings;
        SensorRates m_requestedRates;
        std::vector<std::string> m_hardCodedAhrsOutputStrings;
        ScriptVars m_scriptVars;
        DeviceScript m_onAhrs;
        std::string m_saveConfigPath;
        struct DataFlags
        {
            static const uint_t ahrs = 1 << 0;
            static const uint_t gyro = 1 << 1;
            static const uint_t accel = 1 << 2;
            static const uint_t mag = 1 << 3;
        };

        void connectionEvent(bool_t isConnected) override;
        bool_t newPacket(uint8_t command, const uint8_t* data, uint_t size) override;
        void signalSubscribersChanged(uint_t subscriberCount);
        void logSettings();
        void getData(uint32_t flags);
        void getSettings();
        void getAhrsCal();
        void setGyroCal(uint_t sensorNum, const Vector3* bias);
        void setAccelCal(uint_t sensorNum, const Vector3& bias, const Matrix3x3& transform);
        void setMagCal(uint_t sensorNum, const Vector3& bias, const Matrix3x3& transform);
        void setHeading(const real_t* angleInRadians);
        void clearTurnsCount();
        void getStringNames();
        void getScriptVars();
        void getScript();
        bool_t setScript(const std::string& name, const std::string& code);
    };
}

//--------------------------------------------------------------------------------------------------
#endif
