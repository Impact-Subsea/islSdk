#ifndef SYSPORT_H_
#define SYSPORT_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"
#include "comms/discovery/autoDiscovery.h"
#include "types/sigSlot.h"
#include "comms/protocols/codec.h"
#include "comms/connectionMeta.h"
#include <list>
#include <string>
#include <memory>

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    /**
    * @brief Base class for all system ports.
    * System ports are used to communicate with devices. They are created by the SDK and should not be created directly.
    * They are responsible for running the auto discovery mechanism and encoding / decoding the received frames.
    * @see UartPort
    * @see SolPort
    * @see NetPort
    */
    class SysPort
    {
        friend class Sdk;
        friend class SysPortMgr;
    public:
        typedef std::shared_ptr<SysPort> SharedPtr;

        enum class Type { Serial, Sol, Net };

        const uint32_t id;
        const uint_t discoveryTimeoutMs;                        ///< The default timeout in milliseconds for discovery.
        const std::string name;                                 ///< The name of the port.
        const Type type;                                        ///< The type of the port.
        const bool_t& isOpen = m_isOpen;                        ///< True if the port is open.
        uint_t deviceCount;                                     ///< The number of SDK devices using the port.

        /**
        * @brief A subscribable event for port errors
        * @param port SysPort& The port that triggered the event.
        * @param error std::string The error message.
        */
        Signal<SysPort&, const std::string&> onError;

        /**
        * @brief A subscribable event for when a port is deleted.
        * Ports are deleted when removed from the system like in the case of unplugging a USB serial port.
        * @param port SysPort& The port that triggered the event.
        */
        Signal<SysPort&> onDelete;

        /**
        * @brief A subscribable event for port open
        * @param port SysPort& The port that triggered the event.
        * @param success bool_t True if the port was opened successfully.
        */
        Signal<SysPort&, bool_t> onOpen;

        /**
        * @brief A subscribable event for port close
        * @param port SysPort& The port that triggered the event.
        */
        Signal<SysPort&> onClose;

        /**
        * @brief A subscribable event for port byte counters
        * @param port SysPort& The port that triggered the event.
        * @param txBytes uint_t The number of bytes transmitted.
        * @param rxBytes uint_t The number of bytes received.
        * @param badRxPackets uint_t The number of bad packets received.
        */
        Signal<SysPort&, uint_t, uint_t, uint_t> onPortStats;

        /**
        * @brief A subscribable event for when auto discovery starts on a port.
        * @param port SysPort& The port that triggered the event.
        * @param type AutoDiscovery::Type The type of discovery.
        */
        Signal<SysPort&, AutoDiscovery::Type> onDiscoveryStarted;

        /**
        * @brief A subscribable event for when a device is discovered.
        * @param port SysPort& The port that triggered the event.
        * @param meta const ConnectionMeta& The connection details of the device.
        * @param type AutoDiscovery::Type The type of discovery.
        * @param discoveryCount uint_t The number of devices discovered so far on this port.
        */
        Signal<SysPort&, const ConnectionMeta&, AutoDiscovery::Type, uint_t> onDiscoveryEvent;

        /**
        * @brief A subscribable event for when auto discovery finishes on a port.
        * @param port SysPort& The port that triggered the event.
        * @param type AutoDiscovery::Type The type of discovery.
        * @param discoveryCount uint_t The number of devices discovered on this port.
        * @param cancelled bool_t True if the discovery was cancelled.
        */
        Signal<SysPort&, AutoDiscovery::Type, uint_t, bool_t> onDiscoveryFinished;

        /**
        * @brief A subscribable event for when data is received on a port.
        * @param port SysPort& The port that triggered the event.
        * @param data const uint8_t* The data received.
        * @param size uint_t The size of the data received.
        */
        Signal<SysPort&, const uint8_t*, uint_t> onRxData;

        /**
        * @brief A subscribable event for when data is sent on a port.
        * @param port SysPort& The port that triggered the event.
        * @param data const uint8_t* The data sent.
        * @param size uint_t The size of the data sent.
        */
        Signal<SysPort&, const uint8_t*, uint_t> onTxData;

        SysPort(const std::string& name, Type type, uint_t discoveryTimeoutMs);
        virtual ~SysPort();
        void block(uint32_t lockId);
        void unblock(uint32_t lockId);
        bool_t isBlocked();
        const std::unique_ptr<AutoDiscovery>& getDiscoverer();
        void stopDiscovery();                                       ///< Stops auto discovery.
        bool_t isDiscovering();                                     ///< Returns true if auto discovery is running.
        virtual bool_t open() = 0;
        virtual void close();
        virtual bool_t write(const uint8_t* data, uint_t size, const ConnectionMeta& meta) = 0;
        virtual void discoverIslDevices(uint16_t pid = 0xffff, uint16_t pn = 0xffff, uint16_t sn = 0xffff) {};
        void discoverIslDevices(uint16_t pid, uint16_t pn, uint16_t sn, const ConnectionMeta& meta, uint_t timeoutMs, uint_t count);
        virtual void discoverNmeaDevices() {};

    protected:
        virtual bool_t process();
        void txComplete(const uint8_t* data, uint_t size);
        void nemaDiscovery(const ConnectionMeta& meta, uint_t timeoutMs);
        Callback<SysPort&, const uint8_t*, uint_t, const ConnectionMeta&, Codec::Type> newFrameEvent;
        std::unique_ptr<Codec> m_codec;
        bool_t m_portError;
        bool_t m_isOpen;
        uint32_t m_lock;
        uint_t m_txBytesCount;
        uint_t m_rxBytesCount;
        uint_t m_badRxPacketCount;

    private:
        std::unique_ptr<AutoDiscovery> m_autoDiscoverer;
        bool_t m_sdkCanClose;
    };
}
//--------------------------------------------------------------------------------------------------
#endif
