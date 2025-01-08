#ifndef ISD4000_H_
#define ISD4000_H_

//------------------------------------------ Includes ----------------------------------------------

#include "device.h"
#include "maths/quaternion.h"
#include "maths/vector.h"
#include "files/xmlFile.h"
#include "ahrs.h"
#include <string>
#include <vector>
#include <array>

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    class Isd4000 : public Device
    {
    public:
        class Settings                                  /// Isd4000 Settings information.
        {
        public:
            struct StrOutputSetup
            {
                uint8_t strId;                          ///< Id of the string 0 = script
                bool_t intervalEnabled;                 ///< If true then autonomously aquire and output at the defined interval.
                uint32_t intervalMs;                    ///< Interval in milliseconds to autonomously output.
                Device::CustomStr interrogation;        ///< Custom interrogation string
            };

            Uart::Mode uartMode;                  ///< Serial port mode.
            uint32_t baudrate;                          ///< Serial port baudrate. Limits are standard bauds between 300 and 115200.
            Uart::Parity parity;                      ///< Serial parity.
            uint8_t dataBits;                           ///< Serial word length 5 to 8 bits.
            Uart::StopBits stopBits;                  ///< Serial stop bits.
            uint8_t ahrsMode;                           ///< If bit zero is 1 use inertial mode. 0 is mag slave mode.
            Math::Quaternion orientationOffset;         ///< Heading, pitch and roll offsets (or down and forward vectors) expressed as a quaternion.
            real_t headingOffsetRad;                    ///< Offset in radians to add to the heading. Typically use for magnetic declination.
            Math::Vector3 turnsAbout;                   ///< A vector representing the axis which turn are measured about.
            bool_t turnsAboutEarthFrame;                ///< If true the "turnsAbout" vector is referenced to the earth frame. False is sensor frame.
            Device::CustomStr clrTurn;                  ///< The turns clearing string.
            Device::CustomStr setHeading2Mag;           ///< A string to set the heading to magnetometer heading.
            bool_t filterPressure;                      ///< If true an exponential moving average filter is used to smooth data. This is the filter "output = (lastOutput * 0.9) + (pressure * 0.1)".
            real_t depthOffset;                         ///< Offset in Meters to add to the depth.
            real_t pressureOffset;                      ///< Offset in Bar to add to the calibrated pressure.
            real_t latitude;                            ///< Latitude of the device. Used for gravity accuracy.
            Device::CustomStr tareStr;                  ///< Custom string to tare the pressure.
            Device::CustomStr unTareStr;                ///< Custom string to remove the tare on the pressure.
            StrOutputSetup depthStr;                    ///< depth string setup.    
            StrOutputSetup ahrsStr;					    ///< AHRS string setup.

            static const uint_t size = 271;
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
            uint32_t pressure;                          ///< Interval in milliseconds between pressure data. Zero means no pressure data.
            uint32_t ahrs;                              ///< Interval in milliseconds between AHRS data. Zero means no AHRS data.
            uint32_t gyro;                              ///< Interval in milliseconds between gyro data. Zero means no gyro data.
            uint32_t accel;                             ///< Interval in milliseconds between accelerometer data. Zero means no accelerometer data.
            uint32_t mag;                               ///< Interval in milliseconds between magnetometer data. Zero means no magnetometer data.
            uint32_t temperature;                       ///< Interval in milliseconds between temperature data. Zero means no temperature data.
            SensorRates() : pressure(0), ahrs(0), gyro(0), accel(0), mag(0), temperature(0) {}
        };

        struct CalCert
        {
            uint16_t year;
            uint8_t month;
            uint8_t day;
            uint8_t calPointsLength;
            uint8_t verifyPointsLength;
            std::array<Point, 10> calPoints;
            std::array<Point, 10> verifyPoints;
            std::string number;
            std::string organisation;
            std::string person;
            std::string equipment;
            std::string equipmentSn;
            std::string notes;

            CalCert() : year(0), month(0), day(0), calPointsLength(0), verifyPointsLength(0) {}
            void clear()
            {
                year = 0;
                month = 0;
                day = 0;
                calPointsLength = 0;
                verifyPointsLength = 0;
                number.clear();
                organisation.clear();
                person.clear();
                equipment.clear();
                equipmentSn.clear();
                notes.clear();
            }
        };

        struct AhrsCal
        {
            Math::Vector3 gyroBias;                           ///< Gyro bias corrections in degress per second.
            Math::Vector3 accelBias;                          ///< Accel bias corrections in G.
            Math::Vector3 magBias;                            ///< Mag bias corrections in uT.
            Math::Matrix3x3 accelTransform;                   ///< Transformation matrix for accelerometer.
            Math::Matrix3x3 magTransform;                     ///< Transformation matrix for magnetometer.
        };

        struct PressureCal
        {
            DataState state;
            CalCert cal;
            PressureCal() : state(DataState::Invalid) {}
        };

        struct TemperatureCal
        {
            DataState state;
            CalCert cal;
            int32_t adcOffset;

            TemperatureCal() : state(DataState::Invalid), adcOffset(0) {}
        };

        struct PressureSenorInfo
        {
            
            real_t minPressure;
            real_t maxPressure;

            PressureSenorInfo() : minPressure(0), maxPressure(0) {}
        };


        Ahrs ahrs{ Device::id, this, &Isd4000::setHeading, &Isd4000::clearTurnsCount };             ///< Class to manage AHRS data.
        GyroSensor gyro{ Device::id, 0, this, &Isd4000::setGyroCal };                               ///< Class to manage gyro data.
        AccelSensor accel{ Device::id, 0, this, &Isd4000::setAccelCal };                            ///< Class to manage accelerometer data.
        MagSensor mag{ Device::id, 0, this, &Isd4000::setMagCal, &Isd4000::loadFactoryMagCal };     ///< Class to manage magnetometer data.

        /**
        * @brief A subscribable event for pressure sensor data.
        * @param device Isd4000& The device that triggered the event.
        * @param timeUs uint64_t The time in microseconds.
        * @param pressure real_t The calibrated pressure in Bar.
        * @param depth real_t The calibrated depth in meters.
        * @param pressureRaw real_t The uncalibrated pressure in Bar.
        */
        Signal<Isd4000&, uint64_t, real_t, real_t, real_t> onPressure{ this, &Isd4000::signalSubscribersChanged };

        /**
        * @brief A subscribable event for temperature sensor data.
        * @param device Isd4000& The device that triggered the event.
        * @param temperature real_t The calibrated temperature in degrees C.
        * @param temperatureRaw real_t The uncalibrated temperature in degrees C.
        */
        Signal<Isd4000&, real_t, real_t> onTemperature{ this, &Isd4000::signalSubscribersChanged };

        /**
        * @brief A subscribable event for knowing when the scriptVars, onPing and onAhrs data members are valid.
        * Subscribers to this signal will be called when scriptVars, onPing and onAhrs data has been received.
        * @param device Isa500& The device that triggered the event.
        */
        Signal<Isd4000&> onScriptDataReceived;

        /**
        * @brief A subscribable event for knowing when the settings have been updated.
        * Subscribers to this signal will be called when the settings have been updated.
        * @param device Isa500& The device that triggered the event.
        * @param success bool_t True if the settings were appiled to the device.
        */
        Signal<Isd4000&, bool_t> onSettingsUpdated;

        /**
        * @brief A subscribable event for knowing when the pressure calibration has been received.
        * Subscribers to this signal will be called when the pressure calibration has been received.
        * @param device Isd4000& The device that triggered the event.
        * @param cal const PressureCal& The pressure calibration.
        */
        Signal<Isd4000&, const PressureCal&> onPressureCalCert;

        /**
        * @brief A subscribable event for knowing when the temperature calibration has been received.
        * Subscribers to this signal will be called when the temperature calibration has been received.
        * @param device Isd4000& The device that triggered the event.
        * @param cal const TemperatureCal& The temperature calibration.
        */
        Signal<Isd4000&, const TemperatureCal&> onTemperatureCalCert;

        Isd4000(const Device::Info& info);
        ~Isd4000();

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
        * @brief Sets the depth script with the provided name and code.
        * The script is a custom impact subsea language that is executed on the device and is
        * used to format and build strings to be outputted when used without seaview or the SDK.
        * @param name The name of the depth script.
        * @param code The code for the depth script.
        * @return The boolean value indicating the success of the operation.
        */
        bool_t setDepthScript(const std::string& name, const std::string& code);

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
        * @brief Sets the pressure calibration.
        * @param cert The pressure calibration.
        */
        void setPressureCalCert(const CalCert& cert);

        /**
        * @brief Sets the temperature calibration.
        * @param cert The temperature calibration.
        */
        void setTemperatureCalCert(const CalCert& cert);

        /**
        * @brief Retrieves the pressure and temperature calibration if needed.
        * This function is used to test if retrieval of the pressure and temperature calibration certificates from the device is nessessary.
        * It return's true if the calibrations are valid or false and requests the calibrations from the device.
        * Passing true to the pressure and temperature will force the calibrations to be retrieved from the device.
        * @param pressure A boolean indicating whether to retrieve the pressure calibration.
        * @param temperature A boolean indicating whether to retrieve the temperature calibration.
        * @return The boolean value indicating the success of the operation. If false the calibrations are not valid
        * until an ::onPressureCalCert or ::onTemperatureCalCert event occurs.
        */
        bool_t getCal(bool_t pressure = false, bool_t temperature = false);

        bool_t pressureCalValid() { return m_pressureCal.state == DataState::Valid; }           ///< Returns true if the pressure calibration is valid.
        bool_t temperatureCalValid() { return m_temperatureCal.state == DataState::Valid; }     ///< Returns true if the temperature calibration is valid.

        /**
        * @brief Triggers a measurement of the pressure.
        */
        void measureNow();

        /**
        * @brief Starts logging for the device.
        */
        bool_t startLogging() override;

        /**
        * @brief checks if the firmware has detected hardware faults.
        * @return A vector of the detected hardware faults.
        */
        std::vector<std::string> getHardwareFaults() override;

        /**
        * @brief Saves the configuration with the provided file name.
        * @param fileName The name of the file to save the configuration.
        * @return The boolean value indicating the success of the operation.
        */
        bool_t saveConfig(const std::string& fileName) override;

        /**
        * @brief Returns the configuration as an xml string.
        * @return The boolean value indicating the success of the operation.
        */
        std::string getConfigAsString() override;

        /**
        * @brief Loads the configuration from the provided file name.
        * Any of the pointers can be null if the corresponding data is not required.
        * @param fileName The name of the file from which to load the configuration.
        * @param info Pointer to the Device::Info object to load.
        * @param settings Pointer to the Settings object to load.
        * @param script0 Pointer to the first DeviceScript object to load.
        * @param script1 Pointer to the second DeviceScript object to load.
        * @param cal Pointer to the AhrsCal object to load.
        * @param pCal Pointer to the PressureCal object to load.
        * @param tCal Pointer to the TemperatureCal object to load.
        * @return The boolean value indicating the success of the operation.
        */
        static bool_t loadConfig(const std::string& fileName, Device::Info* info, Settings* settings, DeviceScript* script0, DeviceScript* script1, AhrsCal* cal, PressureCal* pCal, TemperatureCal* tCal);

        bool_t hasAhrs() { return (info.config & 0x01) != 0; }                          ///< Returns true if the device has an AHRS.
        real_t maxPressureRatingBar() { return m_pressureSensorInfo.maxPressure; }      ///< Returns the maximum pressure rating in Bar.

    public:
        const Settings& settings = m_settings;                                          ///< The device settings.
        const SensorRates& sensorRates = m_requestedRates;                              ///< The requested sensor rates.
        const std::vector<std::string>& hardCodedDepthOutputStrings = m_hardCodedDepthOutputStrings;
        const std::vector<std::string>& hardCodedAhrsOutputStrings = m_hardCodedAhrsOutputStrings;
        const ScriptVars& scriptVars = m_scriptVars;                                    ///< The script variables.
        const DeviceScript& onDepth = m_onDepth;                                        ///< The depth script.
        const DeviceScript& onAhrs = m_onAhrs;                                          ///< The AHRS script.
        const PressureCal& pressureCal = m_pressureCal;                                 ///< The pressure calibration.
        const TemperatureCal& temperatureCal = m_temperatureCal;                        ///< The temperature calibration.

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
            SetCalTemperatureAdc,
            GetPressureCal,
            SetPressureCal,
            GetTemperatureCal,
            SetTemperatureCal,
            GetPressureSensorInfo,
        };

        Settings m_settings;
        SensorRates m_requestedRates;
        PressureCal m_pressureCal;
        TemperatureCal m_temperatureCal;
        PressureSenorInfo m_pressureSensorInfo;
        std::vector<std::string> m_hardCodedDepthOutputStrings;
        std::vector<std::string> m_hardCodedAhrsOutputStrings;
        ScriptVars m_scriptVars;
        DeviceScript m_onDepth;
        DeviceScript m_onAhrs;
        std::string m_saveConfigPath;
        struct DataFlags
        {
            static const uint_t pressure = 1 << 0;
            static const uint_t ahrs = 1 << 1;
            static const uint_t gyro = 1 << 2;
            static const uint_t accel = 1 << 3;
            static const uint_t mag = 1 << 4;
            static const uint_t temperature = 1 << 5;
        };

        void connectionEvent(bool_t isConnected) override;
        bool_t newPacket(uint8_t command, const uint8_t* data, uint_t size) override;
        void signalSubscribersChanged(uint_t subscriberCount);
        bool_t logSettings();
        void getData(uint32_t flags);
        void getSettings();
        void getAhrsCal();
        void setGyroCal(uint_t sensorNum, const Math::Vector3* bias);
        void setAccelCal(uint_t sensorNum, const Math::Vector3& bias, const Math::Matrix3x3& transform);
        void setMagCal(uint_t sensorNum, const Math::Vector3& bias, const Math::Matrix3x3& transform, bool_t factory);
        void loadFactoryMagCal();
        void setHeading(const real_t* angleInRadians);
        void clearTurnsCount();
        void getStringNames(uint_t listId);
        void getScriptVars();
        void getScript(uint_t number);
        bool_t setScript(uint_t number, const std::string& name, const std::string& code);
        void setCalCert(Commands command, const CalCert& cert);
        void getPressureSensorInfo();
        bool_t makeXmlConfig(XmlFile& file);
    };
}

//--------------------------------------------------------------------------------------------------
#endif
