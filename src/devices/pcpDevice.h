#ifndef PCPMGR_H_
#define PCPMGR_H_

//------------------------------------------ Includes ----------------------------------------------

#include "pcpServices.h"
#include "comms/ports/poweredComPort.h"
#include "device.h"

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    class PcpDevice : public PcpServices
    {
        friend class MultiPcp;
    public:
        class Settings
		{
        public:
            bool_t powerOn;
            bool_t enabled;
            Uart::Mode portProtocol;
			uint32_t baudrate;
			uint8_t dataBits;
			Uart::Parity parity;
			Uart::StopBits stopBits;

			static const uint_t size = 10;
			Settings();
			void defaults();
			bool_t check(std::vector<std::string>& errMsgs) const;
			uint_t serialise(uint8_t* buffer, uint_t size) const;
			uint_t deserialise(const uint8_t* buffer, uint_t size);
		};
       
        PcpDevice(uint8_t id, Device& device, uint8_t routeCmd);
        ~PcpDevice();
 
        Signal<PcpDevice&, real_t, real_t> onPowerStats;
        Signal<PcpDevice&, bool_t> onSettingsUpdated;

        std::shared_ptr<PoweredComPort> pcp;
        const Settings& settings = m_settings;
        const uint8_t& id = m_id;

        bool_t setSettings(const Settings& settings, bool_t save);
        void setSerial(uint32_t baudrate, uint8_t dataBits, Uart::Parity parity, Uart::StopBits stopBits) override;
        void setSerial(uint32_t baudrate) override;
        bool_t setPower(bool_t on) override;
        void setMode(Uart::Mode portProtocol) override;
        uint32_t getDeviceId() const override;
        uint_t getIndex() const override;
        ConnectionMeta getDeviceConnectionMeta() const override;

    protected:
        uint8_t m_id;
        Device& m_device;
        const uint8_t m_routeCmd;
        bool_t queuePacket(const uint8_t* data, uint_t size);
        virtual void newPacket(uint8_t command, const uint8_t* data, uint_t size);
        
    private:
        Settings m_settings;
        std::unique_ptr<Settings> m_prevSettings;

        enum class Commands
        {
            GetSettings = 20,
            SetSettings,
            Write,
            Read,
        };

        void open() override;
        void close() override;
        bool_t write(const uint8_t* data, uint_t size) override;
        void getSettings();
    };
}

//--------------------------------------------------------------------------------------------------
#endif
