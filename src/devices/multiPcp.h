#ifndef MULTIPCP_H_
#define MULTIPCP_H_

//------------------------------------------ Includes ----------------------------------------------

#include "device.h"
#include "comms/sysPortServices.h"
#include "comms/ports/poweredComPort.h"
#include "pcpDevice.h"

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    class MultiPcp : public Device
    {
    public:
        struct Settings
        {
        public:
            uint32_t ipAddress;
            uint32_t netmask;
            uint32_t gateway;
            uint16_t port;
            bool_t useDhcp;
            Device::PhyPortMode phyPortMode;
            Device::PhyMdixMode phyMdixMode;
 
            static const uint_t size = 17;
            Settings();
            void defaults();
            bool_t check(std::vector<std::string>& errMsgs) const;
            uint_t serialise(uint8_t* buffer, uint_t size) const;
            uint_t deserialise(const uint8_t* buffer, uint_t size);
        };

       
        MultiPcp(const Device::Info& info, SysPortServices& sysPortServices);
        ~MultiPcp();
        bool_t setSettings(const Settings& settings, bool_t save);

        /**
        * @brief Returns the configuration as an xml string.
        * @return The boolean value indicating the success of the operation.
        */
        std::string getConfigAsString() override;

        Signal<MultiPcp&, bool_t> onSettingsUpdated;
        const Settings& settings = m_settings;
        const std::vector<std::shared_ptr<PcpDevice>>& pcpDevices = m_pcpDevices;
          
    private:
        enum class Commands
        {
            GetSettings = 10,
            SetSettings,
            PowerStats,
            RouteMsg,
        };

        Settings m_settings;    
        SysPortServices& m_sysPortServices;
        std::vector<std::shared_ptr<PcpDevice>> m_pcpDevices;

        void connectionEvent(bool_t isConnected) override;
        void raiseConnectionEvent();
        bool_t newPacket(uint8_t command, const uint8_t* data, uint_t size) override;
        void getSettings();
    };
}

//--------------------------------------------------------------------------------------------------
#endif
