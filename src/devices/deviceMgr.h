#ifndef DEVICEMGR_H_
#define DEVICEMGR_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"
#include "device.h"
#include "comms/sysPortServices.h"
#include "comms/ports/sysPort.h"
#include <list>

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    class DeviceMgr                    /// This class manages all the devices.
    {
        friend class Sdk;
    public:
        const std::list<Device::SharedPtr>& deviceList = m_deviceList;                        ///< A read only list of the discovered devices.

        /**
         * @brief Sets the communication timeouts.
         * This function sets the communication timeouts for the host and the device.
         * @param hostCommsTimeoutMs The host communication timeout in milliseconds.
         * @param deviceCommsTimeoutMs The device communication timeout in milliseconds.
         * @param txRetries The number of retries for a failed transmission.
         * @note Device timeouts \p "deviceCommsTimeoutMs" are only set when the device connects.
         * calling this function will not change the devices timeouts of already connected devices.
         */
        void setCommsTimeouts(uint32_t hostCommsTimeoutMs, uint32_t deviceCommsTimeoutMs, uint_t txRetries);

        /**
        * @brief A subscribable event for newly discovered devices.
        * This event is triggered when a new device is discovered.
        * @param device const Device::SharedPtr& The newly discovered device.
        * @param sysPort const SysPort::SharedPtr& The system port the device was discovered on.
        * @param connectionMeta const ConnectionMeta& The connection details e.g. baud rate, ip address etc.
        */
        Signal<const Device::SharedPtr&, const SysPort::SharedPtr&, const ConnectionMeta&> onNew;

        /**
        * @brief Search the classes \p m_deviceList for a device using its id.
        * @param id The id of the device to find.
        * @return The device if found, otherwise nullptr.
        */
        const Device::SharedPtr findById(uint_t id) const;

        /**
        * @brief Search the classes \p m_deviceList for a device using its part number and serial number.
        * @param pn The part number of the device to find.
        * @param sn The serial number of the device to find.
        * @return The device if found, otherwise nullptr.
        */
        const Device::SharedPtr findByPnSn(uint16_t pn, uint16_t sn) const;

        /**
        * @brief Deletes a device from the SDK.
        * This function deletes a device from the SDK. The device will be removed from the \p m_deviceList
        * and raise disconnected and delete events.
        * @param device The device to delete.
        */
        void remove(Device& device);

    private:
        DeviceMgr(SysPortServices& sysPortServices);
        const Device::SharedPtr findByAddress(uint8_t address) const;
        void removePortFromAll(SysPort& sysPort);
        void run();
        Device::SharedPtr createDevice(const Device::Info& deviceInfo);
        SysPortServices& m_sysPortServices;
        std::list<Device::SharedPtr> m_deviceList;
        uint64_t m_timer;
        uint32_t m_hostCommsTimeoutMs;
        uint32_t m_deviceCommsTimeoutMs;
        uint_t m_txRetries;
        std::list<Device::SharedPtr>::iterator deleteDevice(std::list<Device::SharedPtr>::iterator it);
    };
}
//--------------------------------------------------------------------------------------------------
#endif
