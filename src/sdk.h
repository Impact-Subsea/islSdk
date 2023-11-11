#ifndef SDKINTERFACE_H_
#define SDKINTERFACE_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"
#include "comms/sysPortMgr.h"
#include "devices/deviceMgr.h"
#include "nmeaDevices/nmeaDeviceMgr.h"
#include "types/sigSlot.h"

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    /**
    * @brief The main class of the SDK.
    * This class is the main interface to the SDK. It is responsible for managing the SDK and all of its components.
    */
    class Sdk
    {
    public:
        const std::string version = "3.1.0";
        DeviceMgr devices;
        SysPortMgr ports;
        NmeaDeviceMgr nmeaDevices;

        /**
        * @brief Create a new instance of the SDK.
        * This function is responsible for creating a new instance of the SDK.
        */
        Sdk();
        ~Sdk();
        /**
        * @brief Executes the main functionality of the SDK.
        * This function is responsible for running the SDK, Here is a basic list of the SDK tasks.
        * 1.    Detection of serial ports.
        * 2.    Reads and decodes data from serial and network ports.
        * 3.    Manage the auto discovery and detection of devices when required.
        * 4.    Manages the devices protocol stack.
        * This is a non blocking function that should be called periodically, every 50 milliseconds is ideal.
        */
        void run();

    private:
        Slot<const SysPort::SharedPtr&> m_slotNewPort{ this, &Sdk::newPort };
        Slot<SysPort&> m_slotPortDeleted{ this, & Sdk::portDeleted };
        Slot<Device&> m_slotDeviceDeleted{ this, & Sdk::deviceDeleted };
        bool_t m_first;
        void newPort(const SysPort::SharedPtr& sysPort);
        void portDeleted(SysPort& sysPort);
        void deviceDeleted(Device& device);
        void newFrameEvent(SysPort& sysPort, const uint8_t* data, uint_t size, const ConnectionMeta& meta, Codec::Type codecType);
    };
}

//--------------------------------------------------------------------------------------------------
#endif
