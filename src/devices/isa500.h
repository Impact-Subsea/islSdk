#ifndef ISA500_H_
#define ISA500_H_

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
    /**
    * @brief This class is created for every ISA500 device discovered.
    *
    * The class manages the ISA500 and provides various functions to configure the device and get data from it.
    * Most of the devices data is provides by a number of signals that can be subscribed to.
    */
    class Isa500 : public Device
    {
    public:

        enum class AnalogueOutMode { None, Voltage, Current };
        enum class EchoAnalyseMode { First, Strongest, Tracking };

        class Settings                                  /// Isa500 Settings.
        {
        public:
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
            uint8_t multiEchoLimit;                     ///< Sets the maximum multi echo limit, range is 0 to 100.
            uint32_t frequency;                         ///< Frequency of operation in Hertz from 50000 to 700000.
            uint16_t txPulseWidthUs;                    ///< Length of the transmit pulse in microseconds ranging from 0 to 500.
            uint8_t txPulseAmplitude;                   ///< Amplitude of the transmit pulse as a percentage ranging from 0 to 100.
            EchoAnalyseMode echoAnalyseMode;            ///< Selects which echo to report back as the chosen one.
            real_t xcThreasholdLow;                     ///< When the return cross correlated signal level drops below this value the end of an echo pulse is realised. Value ranges from 0 to 1 default is 0.4.
            real_t xcThreasholdHigh;                    ///< When the return cross correlated signal level rises above this value the start of an echo pulse is realised. Value ranges from 0 to 1 default is 0.5.
            real_t energyThreashold;                    ///< Minimum enery an echo must have to be reported. Range is 0 to 1.
            real_t speedOfSound;                        ///< Speed of sound in metres per second.
            real_t minRange;                            ///< Minimum range in metres. Distance is one way, transducer to target.
            real_t maxRange;                            ///< Maximum range in metres. Upper limit is 300, distance is one way, transducer to target.
            real_t distanceOffset;                      ///< Offset + or - in metres to add to the final reading.
            bool_t useTiltCorrection;                   ///< If true the echo range to target will be trigonometrically corrected for pitch and roll.
            bool_t useMaxValueOnNoReturn;               ///< If no echo is detected then use the maximum range value as the reading for outputted strings.
            AnalogueOutMode anaMode;                    ///< Mode of the analogue output.
            real_t aOutMinRange;                        ///< Value in metres. "aOutMinRange" and "aOutMinVal" define a point. e.g 3 metres = 3 volt.
            real_t aOutMaxRange;                        ///< Value in meteres. "aOutMaxRange" and "aOutMaxVal" define another point. e.g 10 metres = 10 volt. These 2 points define a straight line which relates range to output value.
            real_t aOutMinVal;                          ///< Volts or milliamps depending on mode.
            real_t aOutMaxVal;                          ///< Volts or milliamps depending on mode.
            struct CustomStr
            {
                uint8_t strId;                          ///< Id of the string 0 = script.
                bool_t intervalEnabled;                 ///< If true then autonomously ping and output at the defined interval.
                uint32_t intervalMs;                    ///< Interval in milliseconds to autonomously output.
                bool_t triggerEnabled;                  ///< If true the device will ping and output when trigged by the TTL input.
                bool_t triggerEdge;                     ///< If true then the action will happen on the rising edge, false = falling edge.
                Device::CustomStr interrogation;        ///< Custom interrogation string.
            } strTrigger[2];

            static const uint_t size = 249;             ///< Size of the packed structure in bytes.
            Settings();
            void defaults();                            ////< Sets the default values.
            bool_t check(std::vector<std::string>& errMsgs) const;    ///< Checks the settings for validity.
            uint_t serialise(uint8_t* buffer, uint_t size) const;
            uint_t deserialise(const uint8_t* buffer, uint_t size);
            bool_t load(const XmlElementPtr& node);
            void save(XmlElementPtr& node) const;
        };

        struct SensorRates
        {
        uint32_t ping;                                  ///< Interval in milliseconds between pings. Zero means no pings.
            uint32_t ahrs;                              ///< Interval in milliseconds between AHRS data. Zero means no AHRS data.
            uint32_t gyro;                              ///< Interval in milliseconds between gyro data. Zero means no gyro data.
            uint32_t accel;                             ///< Interval in milliseconds between accelerometer data. Zero means no accelerometer data.
            uint32_t mag;                               ///< Interval in milliseconds between magnetometer data. Zero means no magnetometer data.
            uint32_t temperature;                       ///< Interval in milliseconds between temperature data. Zero means no temperature data.
            uint32_t voltage;                           ///< Interval in milliseconds between voltage data. Zero means no voltage data.
            SensorRates() : ping(0), ahrs(0), gyro(0), accel(0), mag(0), temperature(0), voltage(0) {}
        };

        struct Echo
        {
            real_t totalTof;                            ///< Total time of flight in seconds to the target and back.
            real_t correlation;                         ///< How well the received echo correlates 0 to 1.
            real_t signalEnergy;                        ///< Normalised energy level of the echo 0 to 1.
            Echo() : totalTof(0), correlation(0), signalEnergy(0) {}
            Echo(real_t totalTof, real_t correlation, real_t signalEnergy) : totalTof(totalTof), correlation(correlation), signalEnergy(signalEnergy) {}
        };

        struct AhrsCal
        {
            Vector3 gyroBias;                           ///< Gyro bias corrections in degress per second.
            Vector3 accelBias;                          ///< Accel bias corrections in G.
            Vector3 magBias;                            ///< Mag bias corrections in uT.
            Matrix3x3 accelTransform;                   ///< Transformation matrix for accelerometer.
            Matrix3x3 magTransform;                     ///< Transformation matrix for magnetometer.
        };

        struct LicenceFlags
        {
            static const uint_t ahrs = 1 << 0;
            static const uint_t ra = 1 << 1;
            static const uint_t fmd = 1 << 2;
            static const uint_t echogram = 1 << 3;
        };

        Ahrs ahrs{Device::id, this, &Isa500::setHeading, &Isa500::clearTurnsCount};         ///< Class to manage AHRS data.
        GyroSensor gyro{ Device::id, 0, this, &Isa500::setGyroCal };                        ///< Class to manage gyro data.
        AccelSensor accel{ Device::id, 0, this, &Isa500::setAccelCal };                     ///< Class to manage accelerometer data.
        MagSensor mag{ Device::id, 0, this, &Isa500::setMagCal };                           ///< Class to manage magnetometer data.

        /**
        * @brief A subscribable event for ping data.
        * Subscribers to this signal will be called when ping data is received. To set the ping data rate use the setSensorRates() function.
        * @param device Isa500& The device that triggered the event.
        * @param time uint64_t A timestamp issued by the devices firmware in microseconds of the ping data.
        * @param selectedIdx uint_t The index of the selected echo as determined by the echoAnalyseMode setting.
        * @param totalEchoCount uint_t The total number of echoes detected. This number may be larger than the number of echoes in the echo array.
        * @param echos const std::vector<Isa500::Echo>& An array of echo readings. The size of the array is limited to the Settings::multiEchoLimit.
        */
        Signal<Isa500&, uint64_t, uint_t, uint_t, const std::vector<Isa500::Echo>&> onEcho{ this, &Isa500::signalSubscribersChanged };

        /**
        * @brief A subscribable event for echogram data.
        * Subscribers to this signal will be called when echogram data is received.
        * @param device Isa500& The device that triggered the event.
        * @param echogram const std::vector<uint8_t>& The echogram data where echogram[0] is aquired at 0 metres and echogram[echogram.size() - 1] is aquired at Settings::maxRange * 2.
        */
        Signal<Isa500&, const std::vector<uint8_t>&> onEchogramData{ this, & Isa500::echogramSignalSubscribersChanged };

        /**
        * @brief A subscribable event for Temperature data.
        * Subscribers to this signal will be called when temperature data is received. To set the temperature data rate use the setSensorRates() function.
        * @param device Isa500& The device that triggered the event.
        * @param temperature real_t The temperature in degrees Celsius.
        */
        Signal<Isa500&, real_t> onTemperature{ this, &Isa500::signalSubscribersChanged };

        /**
        * @brief A subscribable event for Voltage data.
        * Subscribers to this signal will be called when voltage data is received. To set the voltage data rate use the setSensorRates() function.
        * @param device Isa500& The device that triggered the event.
        * @param voltage real_t The voltage in volts.
        */
        Signal<Isa500&, real_t> onVoltage{ this, &Isa500::signalSubscribersChanged };

        /**
        * @brief A subscribable event for the TTL trigger input wire to the ISA500.
        * Subscribers to this signal will be called when the TTL trigger input wire to the ISA500 is triggered. ie connected or disconnected to pin5 (digital 0V)
        * @param device Isa500& The device that triggered the event.
        * @param edge bool_t False if the trigger wire is connected to digital 0V. True when disconnected.
        */
        Signal<Isa500&, bool_t> onTrigger;

        /**
        * @brief A subscribable event for knowing when the ::scriptVars, ::onPing and ::onAhrs data members are valid.
        * Subscribers to this signal will be called when ::scriptVars, ::onPing and ::onAhrs data has been received.
        * @param device Isa500& The device that triggered the event.
        */
        Signal<Isa500&> onScriptDataReceived;

        /**
        * @brief A subscribable event for knowing when the settings have been updated.
        * Subscribers to this signal will be called when the settings have been updated.
        * @param device Isa500& The device that triggered the event.
        * @param success bool_t True if the settings were appiled to the device.
        */
        Signal<Isa500&, bool_t> onSettingsUpdated;


        Isa500(const Device::Info& info);
        ~Isa500();

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
        * @brief Initiates a ping operation immediately.
        */
        void pingNow();

        /**
        * @brief Sets the number of points for the echogram data.
        * @param dataPointCount The number of data points in the echo gram.
        * @note The echogram is only available if the device has the echogram licence.
        */
        void setEchoGram(uint_t dataPointCount);

        /**
        * @brief Sets the ping script with the provided name and code.
        * The script is a custom impact subsea language that is executed on the device and is
        * used to format and build strings to be outputted when used without seaview or the SDK.
        * @param name The name of the ping script.
        * @param code The code for the ping script.
        * @return The boolean value indicating the success of the operation.
        */
        bool_t setPingScript(const std::string& name, const std::string& code);

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
        * @param script1 Pointer to the second DeviceScript object to load.
        * @param cal Pointer to the AhrsCal object to load.
        * @return The boolean value indicating the success of the operation.
        */
        static bool_t loadConfig(const std::string& fileName, Device::Info* info, Settings* settings, DeviceScript* script0, DeviceScript* script1, AhrsCal* cal);

        /**
        * @brief Starts logging for the Isa500.
        */
        void startLogging() override;

        bool_t hasAhrs() { return (info.config & LicenceFlags::ahrs) != 0; }                        ///< Returns true if the device has the AHRS licence.
        bool_t hasEchoGram() { return (info.config & LicenceFlags::echogram) != 0; }                ///< Returns true if the device has the echogram licence.
        bool_t hasFmd() { return (info.config & LicenceFlags::fmd) != 0; }                          ///< Returns true if the device has the FMD licence.
        bool_t hasRightAngleTransducer() { return (info.config & LicenceFlags::ra) != 0; }          ///< Returns true if the device has a right angle transducer.
        bool_t hasCurrentLoop() { return (info.mode & 0x02) == 0; }                                 ///< Returns true if the device has current loop analogue output.

        const Settings& settings = m_settings;                                                      ///< The current settings.
        const SensorRates& sensorsRates = m_requestedRates;                                         ///< The current sensor rates.
        const std::vector<std::string>& hardCodedPingOutputStrings = m_hardCodedPingOutputStrings;
        const std::vector<std::string>& hardCodedAhrsOutputStrings = m_hardCodedAhrsOutputStrings;
        const ScriptVars& scriptVars = m_scriptVars;                                                ///< The variables available to the script.
        const DeviceScript& onPing = m_onPing;                                                      ///< The ping script.
        const DeviceScript& onAhrs = m_onAhrs;                                                      ///< The AHRS script.

    private:
        enum Commands
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
            
            EchogramData = 28,
        };

        Settings m_settings;
        std::unique_ptr<Settings> m_pendingSettings;
        SensorRates m_requestedRates;
        uint_t m_echogramDataPointCount;
        std::vector<std::string> m_hardCodedPingOutputStrings;
        std::vector<std::string> m_hardCodedAhrsOutputStrings;
        ScriptVars m_scriptVars;
        DeviceScript m_onPing;
        DeviceScript m_onAhrs;
        std::string m_saveConfigPath;
        struct DataFlags
        {
            static const uint_t ping = 1 << 0;
            static const uint_t ahrs = 1 << 1;
            static const uint_t gyro = 1 << 2;
            static const uint_t accel = 1 << 3;
            static const uint_t mag = 1 << 4;
            static const uint_t temperature = 1 << 5;
            static const uint_t voltage = 1 << 6;
            static const uint_t trigger = 1 << 7;
        };

        void connectionEvent(bool_t isConnected) override;
        bool_t newPacket(uint8_t command, const uint8_t* data, uint_t size) override;
        void signalSubscribersChanged(uint_t subscriberCount);
        void echogramSignalSubscribersChanged(uint_t subscriberCount);
        void logSettings();
        void getData(uint32_t flags);
        void getSettings();
        void getAhrsCal();
        void setGyroCal(uint_t sensorNum, const Vector3* bias);
        void setAccelCal(uint_t sensorNum, const Vector3& bias, const Matrix3x3& transform);
        void setMagCal(uint_t sensorNum, const Vector3& bias, const Matrix3x3& transform);
        void setHeading(const real_t* angleInRadians);
        void clearTurnsCount();
        void getStringNames(uint_t listId);
        void getScriptVars();
        void getScript(uint_t number);
        bool_t setScript(uint_t number, const std::string& name, const std::string& code);
    };
}

//--------------------------------------------------------------------------------------------------
#endif
