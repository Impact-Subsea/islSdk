#ifndef DEVICE_H_
#define DEVICE_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"
#include "comms/islHdlc.h"
#include "comms/ports/sysPort.h"
#include "types/sigSlot.h"
#include "logging/loggingDevice.h"
#include <list>
#include <vector>

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    enum class DataState { Invalid, Pending, Valid };

    class ScriptVars
    {
    public:
        class Var
        {
        public:
            enum class Type { Byte, Int, Real, ByteArray, IntArray, RealArray };
            const Type type;
            const std::string name;
            const std::string description;
            Var(const std::string& name, const std::string& description, ScriptVars::Var::Type type) : type(type), name{ name }, description{ description } {}
        };
        DataState state;
        std::vector<Var> vars;
        ScriptVars() : state(DataState::Invalid) {}
    };

    class DeviceScript
    {
    public:
        DataState state;
        std::string name;
        std::string code;
        DeviceScript() : state(DataState::Invalid) {};
        DeviceScript(const std::string& name, const std::string& code) : state(DataState::Valid), name{ name }, code{ code } {};
    };

    /**
    * @brief The base class for all devices.
    * This class is the base class for all device types. It provides the basic functionality for
    * connecting to a device and sending/receiving data.
    */
    class Device : public IslHdlc, public LoggingDevice
    {
        friend class Sdk;
        friend class DeviceMgr;

    public:
        typedef std::shared_ptr<Device> SharedPtr;

        enum class Pid                      ///< Product id values.
        {
            Unknown = 0,
            Isa500v1 = 1242,                ///< ISA500 with firmware less than V3.x.x This SDK supports firmware V3.0.0 and above.
            Isd4000v1 = 1299,               ///< ISD4000 with firmware less than V3.x.x This SDK supports firmware V3.0.0 and above.
            Ism3dv1 = 1444,                 ///< ISM3D with firmware less than V3.x.x This SDK supports firmware V3.0.0 and above.
            Iss360v1 = 1660,                ///< ISS360 with firmware less than V3.x.x This SDK supports firmware V3.0.0 and above.
            Isa500 = 1336,                  ///< ISA500 with firmware V3.0.0 and above.
            Isd4000 = 1342,                 ///< ISD4000 with firmware V3.0.0 and above.
            Ism3d = 1461,                   ///< ISM3D with firmware V3.0.0 and above.
            Sonar = 1791,                   ///< ISS360, ISS360HD or ISP360 with firmware V3.0.0 and above.
            Any = 0xffff,
        };

        enum class UartMode : uint8_t { Rs232, Rs485, Rs485Terminated, Unknown = 255};
        enum class Parity : uint8_t { None, Odd, Even, Mark, Space, Unknown = 255};
        enum class StopBits : uint8_t { One, OneAndHalf, Two, Unknown  = 255};
        enum class PhyMdixMode : uint8_t { Normal, Swapped, Auto, Unknown = 255};
        enum class PhyPortMode : uint8_t { Auto, Base10TxHalf, Base10TxFull, Base100TxHalf, Base100TxFull, Unknown = 255 };

        /**
        * @brief The device information structure.
        * All devices send this data on discovery.
        */
        class Info
        {
        public:
            static const uint_t size = 14;
            Pid pid;                                ///< Product id.
            uint16_t pn;                            ///< Part number as shown on the device lable.
            uint16_t sn;                            ///< Serial number as shown on the device lable.
            uint8_t config;                         ///< Configuration value. Has different meaning for different devices.
            uint8_t mode;                           ///< Mode value. Has different meaning for different devices.
            uint16_t status;                        ///< Status value. Has different meaning for different devices.
            uint16_t firmwareBuildNum;              ///< Firmware build number.
            uint16_t firmwareVersionBcd;            ///< Firmware revision. Each nibble represents a digit, e.g 0x1234 would be V1.2.3.4.
            bool_t inUse;                           ///< If true device is connected to another host.

            Info();
            Info(const uint8_t* data);
            void fromBuf(const uint8_t* data);
            bool_t isDifferent(const Info& d) const;
            std::string pnSnAsStr() const;          ///< Form a string from pn and sn as shown on the lable.
            std::string name() const;               ///< Form a name string from pid.
        };

        class CustomStr
        {
        public:
            static const uint_t size = 32;
            bool_t enable;
            std::string str;
            CustomStr() : enable(false) {}
            CustomStr(bool_t enable, const std::string& str) : enable(false), str(str) {}
            uint_t packStr(uint8_t* ptr) const
            {
                for (uint_t i = 0; i < size; i++)
				{
                    *ptr++ = i < str.size() ? str[i] : 0;
				}
                return size;
            }
        };

        const uint_t& reconnectCount = m_reconnectCount;                ///< The number of times the device has been reconnected.
        const Info& info = m_info;                                      ///< The device information.
        const std::unique_ptr<Connection>& connection = m_connection;   ///< The connection object.

        /**
        * @brief A subscribable event for device errors
        * @param device Device& The device that triggered the event.
        * @param error std::string The error message.
        */
        Signal<Device&, const std::string&> onError;

        /**
        * @brief A subscribable event for when the device is deleted.
        * This event is triggered when the SDK deletes the device. This happens when the device is
        * no longer connected to the host and the SDK has not been able to reconnect to it.
        * @param device Device& The device that triggered the event.
        */
        Signal<Device&> onDelete;

        /**
        * @brief A subscribable event for when a device connects.
        * This event is triggered when the device connects to the host.
        * @param device Device& The device that triggered the event.
        */
        Signal<Device&> onConnect;

        /**
        * @brief A subscribable event for when a device disconnects.
        * This event is triggered when the device disconnects from the host.
        * @param device Device& The device that triggered the event.
        */
        Signal<Device&> onDisconnect;

        /**
        * @brief A subscribable event for when a communication port is added to a device.
        * The communication port is presented as the base class SysPort. The derived class represents
        * the type of port (e.g. serial or network), and this is the port the device was discovered on.
        * @param device Device& The device that triggered the event.
        * @param port SysPort& The port that was added and the device was discovered on.
        * @param meta ConnectionMeta& The baudrate or IP address and port the device was discovered on.
        */
        Signal<Device&, SysPort&, const ConnectionMeta&> onPortAdded;

        /**
        * @brief A subscribable event for when a communication port connection info is changed.
        * If the device connection details for the port (e.g baudrate for serial or ip address for network)
        * are changed this event is triggered.
        * @param device Device& The device that triggered the event.
        * @param port SysPort& The port the device is communicating on.
        * @param meta ConnectionMeta& The new baudrate or IP address and port.
        */
        Signal<Device&, SysPort&, const ConnectionMeta&> onPortChanged;

        /**
        * @brief A subscribable event for when a communication port is removed from a device.
        * This event fires when the communication port is removed from a device, such as when the
        * device is deleted or the port is no longer available.
        * @param device Device& The device that triggered the event.
        * @param port SysPort& The port that was removed and the device was discovered on.
        */
        Signal<Device&, SysPort&> onPortRemoved;

        /**
        * @brief A subscribable event for when the device information is changed.
        * This event is triggered when the device information is changed. This event is
        * rare and only fires when licences are added.
        * @param device Device& The device that triggered the event.
        * @param info Device::Info& The new device information.
        */
        Signal<Device&, const Device::Info&> onInfoChanged;

        /**
        * @brief A subscribable event that fires every second and reports the packet counters for the device
        * The counters are reset to zero when the device connects.
        * @param device Device& The device that triggered the event.
        * @param txCount uint64_t The number of packets transmitted.
        * @param rxCount uint64_t The number of packets received.
        * @param resends uint64_t The number of packets re-transmitted.
        * @param missed uint64_t The number of packets that failed to be received.
        */
        Signal<Device&, uint_t, uint_t, uint_t, uint_t> onPacketCount;

        Device(const Device::Info& info);
        virtual ~Device();

        /**
        * @brief Connect to a device.
        * This function connects to the device. If the device is already connected it has no effect.
        * This established a connection session between the host and device.
        */
        void connect();

        /**
        * @brief Setup how long a device should be searched for when it loses comms.
        * When a device loses comms with the host it will disconnect and trigger a rediscover the device.
        * This function sets how long the host should wait for a responce to it's discovery message and
        * the number of attempts before giving up.
        * @param searchTimeoutMs The time in milliseconds to search for the device.
        * @param searchCount The number of times to search for the device.
        * @note searchCount is ignored and set to 1 attempt if the device is on an RS485 multi-drop bus.
        */
        void setRediscoveryTimeouts(uint_t searchTimeoutMs, uint_t searchCount);

        /**
        * @brief Set the number of times a packet should be re-sent before giving up and disconnecting.
        * If the protocol stack detects a failed transmission it will attempt to re-send the packet.
        * The timeout for acknowledged reception is exponential increased with every attempt, i.e. the first re-send is after 1000ms, the second after 2000ms, the third after 4000ms etc.
        * @param retries The number of times to attempt a re-send of the packet before deciding the device has disconnected.
        */
        void setCommsRetries(uint_t retries);

        /**
        * @brief Reset the device.
        * The device is instructed to reset.
        */
        void reset();

        /**
        * @brief Save the device configuration to an xml file.
        * This function saves all device parameters to an xml file.
        * @param fileName The name of the file to save the configuration to.
        * @return True if the configuration was saved successfully.
        */
        virtual bool_t saveConfig(const std::string& fileName) { return false; }

        /**
        * @brief Check if the devices is in bootloader mode.
        * @return True if the device is in bootloader mode.
        */
        bool_t bootloaderMode() { return (info.mode & 0x01) != 0; }
        

    protected:
        enum class Commands { Reset = 1, Descriptor, ReplyBit = 0x80 };
        virtual void connectionEvent(bool_t connected);
        virtual bool_t newPacket(uint8_t command, const uint8_t* data, uint_t size) { return false; }
        void enqueuePacket(const uint8_t* data, uint_t size);
        void sendPacket(const uint8_t* data, uint_t size);
        void connectionSettingsUpdated(const ConnectionMeta& meta, bool_t isHalfDuplex);
        bool_t startLogging() override;
        bool_t m_connectionDataSynced;
        uint64_t m_epochUs;

    private:
        std::unique_ptr<ConnectionMeta> m_pendingMeta;
        uint_t m_packetResendLimit;
        uint32_t m_deviceTimeOut;
        Device::Info m_info;
        uint_t m_reconnectCount;
        bool_t m_resetting;
        uint_t m_searchTimeoutMs;
        uint_t m_searchCount;
        uint64_t m_deleteTimer;


        bool_t shouldDelete() const;
        void addOrUpdatePort(const SysPort::SharedPtr& sysPort, const ConnectionMeta& meta);
        void removePort();
        void updateConnection(const ConnectionMeta& meta);
        uint_t reDiscover(uint_t timeoutMs, uint_t count);
        void hdlcConnectionEvent(bool_t connected) override;
        bool_t timeoutEvent() override;
        void newPacketEvent(const uint8_t* data, uint_t size) override;
        LoggingDevice::Type getTrackData(std::vector<uint8_t>& buf) override;
        void logData(uint8_t dataType, const std::vector<uint8_t> data) override;
        
    };
}

//--------------------------------------------------------------------------------------------------

#endif
