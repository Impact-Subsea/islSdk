#ifndef SONAR_H
#define SONAR_H

//------------------------------------------ Includes ----------------------------------------------

#include "device.h"
#include "ahrs.h"
#include "maths/quaternion.h"
#include "maths/vector.h"
#include "maths/matrix.h"
#include "files/xmlFile.h"
#include <string>
#include <array>
#include <vector>

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    class Sonar : public Device
    {
    public:
        static const uint_t maxAngle = 12800;
        static real_t countToDegrees(int_t count)
        { 
            return count * (360.0 / static_cast<real_t>(maxAngle));
        }

        static int_t degreesToCount(real_t angle)
        {
            return static_cast<int_t>((angle * (static_cast<real_t>(maxAngle) / 360.0)) + 0.5);
        }

        enum class Type { Iss360HD, Iss360, Isp360Profiler, Unknown = 255};

        struct Sector
        {
            uint_t start;
            uint_t size;
            Sector() : start(0), size(0) {}
        };

        struct System                           ///< System Settings for the device.
        {
        public:
            Uart::Mode uartMode;                ///< Serial port mode.
            uint32_t baudrate;                  ///< Serial port baudrate. Limits are standard bauds between 9600 and 921600
            uint32_t ipAddress;                 ///< IPv4 address. 192.168.1.200 = 0xc801A8c0
            uint32_t netmask;                   ///< IPv4 netmask. 255.255.255.0 = 0x00ffffff
            uint32_t gateway;                   ///< IPv4 gateway. 192.168.1.1 = 0x0101A8c0
            uint16_t port;                      ///< Port the device listens on and transmits from
            Device::PhyPortMode phyPortMode;    ///< Ethernet connection speed anf duplex mode
            Device::PhyMdixMode phyMdixMode;    ///< Ethernet TX/RX swapping mode
            bool_t useDhcp;                     ///< If true device will request an IP address from the DHCP server
            bool_t invertHeadDirection;         ///< If true the head direction is swapped
            uint8_t ahrsMode;                   ///< If bit zero is 1 use inertial mode. 0 is mag slave mode
            Math::Quaternion orientationOffset; ///< Heading, pitch and roll offsets (or down and forward vectors) expressed as a quaternion.
            real_t headingOffsetRad;            ///< Offset in radians to add to the heading. Typically use for magnetic declination.
            Math::Vector3 turnsAbout;           ///< A vector representing the axis which turn are measured about.
            bool_t turnsAboutEarthFrame;        ///< If true the "turnAbout" vector is referenced to the earth frame. False is sensor frame.
            bool_t useXcNorm;                   ///< Output normalised cross correlation data instead of unnormalised. Normalised data represents the quality of the echo rather than the strength of the echo.
            bool_t data8Bit;                    ///< true = 8-bit data, false = 16-bit data
            enum class GatingMode { Off, On, RollComp } gatingMode;    ///< Gating mode for the sonar. Off = no gating, On = gating at an angle to the sector center, RollComp = gating at an angle to the sector center and compensating for roll.
            int16_t gatingAngle;                ///< The gating angle to a line perpendicular to the sector center in units of 12800th. 360 degrees = a value of 12800.
            real_t speedOfSound;                ///< Speed of sound in meters per second. limits 1000 to 2500

            System();
            void defaults();
            bool_t check(std::vector<std::string>& errMsgs) const;
            uint_t serialise(uint8_t* buffer, uint_t size) const;
            uint_t deserialise(const uint8_t* buffer, uint_t size);
            bool_t load(const XmlElementPtr& node);
            void save(XmlElementPtr& node) const;

            static const uint_t size = 63;
        };

        struct Acoustic                         ///< Acoustic Settings for the device.
        {
            uint32_t txStartFrequency;          ///< Transmit pulse start frequency in hertz.
            uint32_t txEndFrequency;            ///< Transmit pulse end frequency in hertz.
            uint16_t txPulseWidthUs;            ///< Transmit pulse length in micro seconds.
            uint8_t txPulseAmplitude;           ///< Transmit pulse amplitude as a percent 0% to 100%
            bool_t highSampleRate;              ///< If true the ADC sample rate is 5 MHz else it's 2.5 MHz
            uint32_t pskCode;                   ///< PSK code for the transmit pulse. 0 to 31
            uint8_t pskLength;                  ///< Length of the PSK code. 0 to 31

            static const uint_t size = 17;

            Acoustic();
            void defaults();
            bool_t check(std::vector<std::string>& errMsgs) const;
            uint_t serialise(uint8_t* buffer, uint_t size) const;
            uint_t deserialise(const uint8_t* buffer, uint_t size);
            bool_t load(const XmlElementPtr& node);
            void save(XmlElementPtr& node) const;
        };

        struct Setup                            ///< Setup Settings for the device.
        {
            int32_t stepSize;                   ///< Angle the tranducer head should move between pings in units of 12800th. Positive values turn clockwise, negative anticlockwise. limits -6399 to 6399
            uint32_t sectorStart;               ///< Start angle of the sector. limmts 0 to 12799
            uint32_t sectorSize;                ///< Size of the sector. limits 0 to 12800
            bool_t flybackMode;                 ///< If true the transducer head returns back to either the \p sectorStart position when \p stepSize is positive, or s\p sectorStart + \p sectorSize when \p stepSize is negative
            uint16_t imageDataPoint;            ///< Number of data points per ping between the range set by minRangeMm and maxRangeMm. limits 20 to 4096
            uint32_t minRangeMm;                ///< Start listening for echos after this range in millimeters.
            uint32_t maxRangeMm;                ///< Listen for echos up until this range in millimeters.
            uint32_t profilerMinRangeMm;        ///< Applies to profiling mode only. Start listening for echos after this range in millimeters.
            uint32_t profilerMaxRangeMm;        ///< Applies to profiling mode only. Listen for echos up until this range in millimeters
            real_t digitalGain;                 ///< Digital gain for the image data as a simple multiplier factor. limits 1 to 1000
            enum class EchoMode { First, Strongest, All } echoMode;        ///< Applies to profiling mode only. Selects which echo to report back as the chosen one when profiling.
            real_t xcThreasholdLow;             ///< Applies to profiling mode only. Sets a lower limit on the quality of the return pulse. This ensures resilience to false echos. Value ranges from 0 to 1.
            real_t xcThreasholdHigh;            ///< Applies to profiling mode only. When the return signal level drops bellow this value the end of an echo pulse is realised. Value ranges from 0 to 1.
            real_t energyThreashold;            ///< Applies to profiling mode only. Minimum enery an echo must have to be reported. Range is 0 to 1    
            
            static const uint_t size = 48;

            Setup();
            void defaults();
            bool_t check(std::vector<std::string>& errMsgs) const;
            uint_t serialise(uint8_t* buffer, uint_t size) const;
            uint_t deserialise(const uint8_t* buffer, uint_t size);
            bool_t load(const XmlElementPtr& node);
            void save(XmlElementPtr& node) const;
        };

        struct Settings                         ///< Sonar Settings information.
        {
            enum class Type { System, Acoustic, Setup };
            System system;                      ///< System settings
            Acoustic acoustic;                  ///< Acoustic settings
            Setup setup;                        ///< Setup settings

            static const uint_t size = System::size + Acoustic::size + Setup::size;

            Settings();
            void defaults();
            uint_t serialise(uint8_t* buf, uint_t size) const;
            uint_t deserialise(const uint8_t* data, uint_t size);
            bool_t load(const XmlElementPtr& node);
            void save(XmlElementPtr& node) const;
        };

        struct SensorRates
        {
            uint32_t ahrs;                      ///< Interval in milliseconds between AHRS data. Zero means no AHRS data.
            uint32_t gyro;                      ///< Interval in milliseconds between gyro data. Zero means no gyro data.
            uint32_t accel;                     ///< Interval in milliseconds between accelerometer data. Zero means no accelerometer data.
            uint32_t mag;                       ///< Interval in milliseconds between magnetometer data. Zero means no magnetometer data.
            uint32_t voltageAndTemp;            ///< Interval in milliseconds between system voltage and temperature data.
        };

        struct AhrsCal
        {
            Math::Vector3 gyroBias;                   ///< Gyro bias corrections in degress per second.
            Math::Vector3 accelBias;                  ///< Accel bias corrections in G.
            Math::Vector3 magBias;                    ///< Mag bias corrections in uT.
            Math::Matrix3x3 accelTransform;           ///< Transformation matrix for accelerometer.
            Math::Matrix3x3 magTransform;             ///< Transformation matrix for magnetometer.
        };

        struct HeadIndexes					    /// Information about the positions of the optical head marker. These are used to referance the stepper motor home position.
        {
            struct Index
			{
				uint_t idx;					    ///< The position of the index. (Units of 12800th)   
				bool_t level;				    ///< The level of the index. True = low to high transition, false = high to low transition.
                bool_t dir;					    ///< The direction the head was traveling when the index was aquired. True = clockwise, false = anticlockwise.
			};

            enum class State { Ok, ErrRotaion, ErrIdxCount, ErrNoMatch } state;	///< The state of the index acquisition.
            int_t slippage;					    ///< The amount of slippage since the last acquisition or bootup in units of 12800th. 360 degrees = a value of 12800.
            real_t stdDeviation;			    ///< The standard deviation of the indexes in units of 12800th. 360 degrees = a value of 12800.
            int_t hysteresisCorrection;		    ///< The hysterisis correction applied.
            int_t widthCorrection;			    ///< The width correction applied.
            std::vector<Index> indexes;		    ///< A list of the indexes found.
        };

        struct Ping                             /// This is the cross correlation data received from the device on each ping. The data is used to produce a sonar image of the environment.
        {
            uint_t angle;                       ///< Angle the data was aquired at in units of 12800th. 360 degrees = a value of 12800.
            int_t stepSize;                     ///< The step size setting at the time this data was aquired.
            uint_t minRangeMm;                  ///< Start distance of the data in millimeters, \p data[0] is aquired at this range
            uint_t maxRangeMm;                  ///< Final distance of the data in millimeters, \p data[data.size()-1] is aquired at this range
            std::vector<uint16_t> data;         ///< Array of ping data. Each value represents the amplitude of the signal at a range. The range is given by: range = minRangeMm + (arrayIndex * ((maxRangeMm - minRangeMm) / data.size()))
            Ping() : angle(0), stepSize(0), minRangeMm(0), maxRangeMm(0) {}
        };

        struct Echos                            /// This is a list of echos received from the device on each ping. This is the profiling data.
        {
            struct Echo
            {
                real_t totalTof;                ///< Total time of flight in seconds to the target and back
                real_t correlation;             ///< How well the received echo correlates 0 to 1
                real_t signalEnergy;            ///< Normalised energy level of the echo 0 to 1
                Echo() : totalTof(0), correlation(0), signalEnergy(0) {}
                Echo(real_t totalTof, real_t correlation, real_t signalEnergy) : totalTof(totalTof), correlation(correlation), signalEnergy(signalEnergy) {}
            };

            uint64_t timeUs;                    ///< Time in microseconds of the start of the ping
            uint_t angle;                       ///< Angle the data was aquired at in units of 12800th. 360 degrees = a value of 12800
            std::vector<Echo> data;             ///< Array of echos. Each echo represents a single target.
        };

        struct CpuPowerTemp                     /// Power and temperature data.
        {
            real_t core1V0;                     ///< CPU Core voltage, should be 1V � 0.05V.
            real_t aux1V8;                      ///< Auxillary voltage, should be 1.8V � 0.09V.
            real_t ddr1V35;                     ///< DDR voltage, should be 1.35V � 0.05V.
            real_t cpuTemperature;              ///< CPU temperature in degrees C. Should be between 0 and 85.
            real_t auxTemperature;              ///< Auxillary temperature in degrees C. Should be between 0 and 85.

            CpuPowerTemp() : core1V0(0), aux1V8(0), ddr1V35(0), cpuTemperature(0), auxTemperature(0) {}
        };

        

        const Settings& settings = m_settings;                              ///< Current settings of the device.
        const SensorRates& sensorRates = m_requestedRates;                  ///< Current sensor rates of the device.
        const std::array<Point, 9>& tvgPoints = m_tvgPoints;                ///< Current TVG (Time Variable Gain) curve of the device.
        const uint8_t(&macAddress)[6] = (&m_macAddress)[0];                 ///< Mac Address of the ethernet interface. Read only, setting has no effect.

        Ahrs ahrs{ Device::id, this, &Sonar::setHeading, &Sonar::clearTurnsCount };             ///< Class to manage AHRS data.
        GyroSensor gyro{ Device::id, 0, this, &Sonar::setGyroCal };                             ///< Class to manage Gyro data.
        AccelSensor accel{ Device::id, 0, this, &Sonar::setAccelCal };                          ///< Class to manage Accelerometer data.
        MagSensor mag{ Device::id, 0, this, &Sonar::setMagCal, &Sonar::loadFactoryMagCal };    ///< Class to manage Magnetometer data.

        /**
        * @brief A subscribable event for knowing when the settings have been updated.
        * Subscribers to this signal will be called when the settings have been updated.
        * @param device Isa500& The device that triggered the event.
        * @param success bool_t True if the settings were appiled to the device.
        */
        Signal<Sonar&, bool_t, Settings::Type> onSettingsUpdated;

        /**
        * @brief A subscribable event for knowing when the head has finished acquiring the home index after a acquireHeadIdx command.
        * Subscribers to this signal will be called once the head has finished acquiring the home index.
        * @param device Sonar& The device that triggered the event.
`       * @param indexState const HeadIndexes& Information about the head index positions.
        */
        Signal<Sonar&, const HeadIndexes&> onHeadIndexesAcquired;

        /**
        * @brief A subscribable event for knowing when the device has received new ping data.
        * Subscribers to this signal will be called when the device has received new ping data.
        * this information is used to produce a sonar image of the environment.
        * @param device Sonar& The device that triggered the event.
        * @param ping const Ping& The ping data.
        */
        Signal<Sonar&, const Ping&> onPingData{ this, & Sonar::sonarDataSignalSubscribersChanged };

        /**
        * @brief A subscribable event for knowing when the device has received new echo data.
        * Subscribers to this signal will be called when the device has received new echo data.
        * this information is used for profiling as it gives a list of echos received for each ping.
        * @param device Sonar& The device that triggered the event.
        * @param echos const Echos& The echo data.
        */
        Signal<Sonar&, const Echos&> onEchoData{ this, & Sonar::sonarDataSignalSubscribersChanged };

        /**
        * @brief A subscribable event for knowing when the device has received voltage and temperature data.
        * @param device Sonar& The device that triggered the event.
        * @param status const CpuPowerTemp& The voltage and temperature data.
        */
        Signal<Sonar&, const CpuPowerTemp&> onPwrAndTemp{ this, & Sonar::signalSubscribersChanged };

        /**
        * @brief A subscribable event for knowing when the device has detected motor slip.
        * @param device Sonar& The device that triggered the event.
        */
        Signal<Sonar&> onMotorSlip;

        /**
        * @brief A subscribable event for knowing when the device has completed moving the motor.
        * * @param device Sonar& The device that triggered the event.
        * @param success bool_t True if the motor moved successfully.
        */
        Signal<Sonar&, bool_t> onMotorMoveComplete;

        

        Sonar(const Device::Info& info);
        ~Sonar();

        /**
        * @brief Sets the data rates for the device.
        * This function is used to configure the sampling rates for the various sensors used in the system..
        * @param sensors A reference to the 'SensorRates' object containing the desired sampling rates for the sensors.
        */
        void setSensorRates(const SensorRates& sensors);

        /**
        * @brief Sets the device system settings and optionally saves them.
        * @param settings The settings to be applied to the device.
        * @param save A boolean indicating whether to save the settings.
        * @return The boolean value indicating the success of the operation.
        * @note Device::onError event will be called if the settings are invalid.
        * @warning The Flash in this device has an endurance of 100000 write cycles. Writing to the Flash too often will
        * reduce the life of the device. Consider setting save to false if updating the settings frequently.
        */
        bool_t setSystemSettings(const System& settings, bool_t save);

        /**
        * @brief Sets the device acoustic settings and optionally saves them.
        * @param settings The settings to be applied to the device.
        * @param save A boolean indicating whether to save the settings.
        * @return The boolean value indicating the success of the operation.
        * @note Device::onError event will be called if the settings are invalid.
        * @warning The Flash in this device has an endurance of 100000 write cycles. Writing to the Flash too often will
        * reduce the life of the device. Consider setting save to false if updating the settings frequently.
        */
        bool_t setAcousticSettings(const Acoustic& settings, bool_t save);

        /**
        * @brief Sets the device setup settings and optionally saves them.
        * @param settings The settings to be applied to the device.
        * @param save A boolean indicating whether to save the settings.
        * @return The boolean value indicating the success of the operation.
        * @note Device::onError event will be called if the settings are invalid.
        * @warning The Flash in this device has an endurance of 100000 write cycles. Writing to the Flash too often will
        * reduce the life of the device. Consider setting save to false if updating the settings frequently.
        */
        bool_t setSetupSettings(const Setup& settings, bool_t save);

        /**
        * @brief Set the head to it's home position.
        * @note The ::onHeadIndexesAcquired event will be called when the head has finished homing.
        */
        void acquireHeadIdx(bool_t fullSync=false);

        /**
        * @brief Start scanning.
        * This will start the device scanning and sending data back to the SDK.
        */
        void startScanning();

        /**
        * @brief Stop scanning.
        */
        void stopScanning();

        /**
        * @brief Move the head to a position.
        * @param angle The angle to move the head to or by in units of 12800th. 360 degrees = a value of 12800.
        * @param relative If true the head will move by the angle specified, if false the head will move to the angle specified.
        */
        void moveHead(int16_t angle, bool_t relative = false);

        /**
        * @brief This sets the sonar to ouput checker board patterned ping data for testing.
        * @param enable If true the sonar will output checker board patterned ping data.
        */
        void testPattern(bool_t enable);

        /**
        * @brief Sets the TVG (Time Variable Gain) curve.
        * The TVG curve is computed from these points using piecewise cubic spline interpolation.
        * @param points An array of 9 points defining the shape of the TVG curve.
        */
        void setTvg(const std::array<Point, 9>& points);

        /**
        * @brief Gets the default TVG (Time Variable Gain) curve.
        * @return An array of 9 points defining the shape of the TVG curve.
        */
        static std::array<Point, 9> getDefaultTvg();
        

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
        * @param cal Pointer to the AhrsCal object to load.
        * @param tvgPoints Pointer to the array of TVG points to load.
        * @return The boolean value indicating the success of the operation.
        */
        static bool_t loadConfig(const std::string& fileName, Device::Info* info, Settings* settings, AhrsCal* cal, std::array<Point, 9>* tvgPoints);

        Type getType();
        bool_t hasAhrs() { return (info.config & 0x01) != 0; }              ///< Returns true if the device has an AHRS.

    private:

        enum class Commands
        {
            GetSensorData = 10,
            SetSensorInterval,
            GetSettings,
            SaveSettings,
            SetSystemSettings,
            SetAcousticSettings,
            SetSetupSettings,
            GetAhrsCal,
            SetGyroCal,
            SetAccelCal,
            SetMagCal,
            SetHeading,
            ClearTurnsCount,
            PingData,
            EchoData,
            AcquireHeadIdx,
            StopStart,
            SetDataOptions,
            MoveMotor,
            TestImage,
            GetTvg,
            SetTvg,
            
        };

        Settings m_settings;
        SensorRates m_requestedRates;
        std::string m_saveConfigPath;
        std::array<Point, 9> m_tvgPoints;
        uint8_t m_macAddress[6];
        struct DataFlags
        {
            static const uint_t ahrs = 1 << 0;
            static const uint_t gyro = 1 << 1;
            static const uint_t accel = 1 << 2;
            static const uint_t mag = 1 << 3;
            static const uint_t cpuTempPower = 1 << 4;
            static const uint_t motorSlip = 1 << 5;
        };

        void connectionEvent(bool_t isConnected) override;
        bool_t newPacket(uint8_t command, const uint8_t* data, uint_t size) override;
        void signalSubscribersChanged(uint_t subscriberCount);
        void sonarDataSignalSubscribersChanged(uint_t subscriberCount);
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
        void selectDataOutput(bool_t ping, bool_t echo);
        void getTvg();
        bool_t makeXmlConfig(XmlFile& file);
    };
}

//--------------------------------------------------------------------------------------------------
#endif
