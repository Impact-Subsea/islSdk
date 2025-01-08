//------------------------------------------ Includes ----------------------------------------------

#include "multiPcp.h"
#include "utils/stringUtils.h"
#include "utils/utils.h"
#include "platform/debug.h"

using namespace IslSdk;

//--------------------------------------------------------------------------------------------------
MultiPcp::MultiPcp(const Device::Info& info, SysPortServices& sysPortServices) : Device(info), m_sysPortServices(sysPortServices)
{
    uint8_t portCount = info.mode >> 4;

    if (info.pid == Device::Pid::MultiPcp)
    {
        for (uint8_t i = 0; i < portCount; i++)
        {
            std::shared_ptr<PcpDevice> pcpDevice = std::make_shared<PcpDevice>(i, *this, static_cast<uint8_t>(Commands::RouteMsg));
            m_pcpDevices.push_back(pcpDevice);
        }
    } 
}
//--------------------------------------------------------------------------------------------------
MultiPcp::~MultiPcp()
{
}
//--------------------------------------------------------------------------------------------------
bool_t MultiPcp::setSettings(const Settings& newSettings, bool_t save)
{
    uint8_t data[Settings::size + 2];
    std::vector<std::string> errMsgs;
    bool_t ok = newSettings.check(errMsgs);

    if (ok)
    {
        data[0] = static_cast<uint8_t>(Commands::SetSettings);
        data[1] = static_cast<uint8_t>(save);
        newSettings.serialise(&data[2], sizeof(data) - 2);

        if (((newSettings.ipAddress != m_settings.ipAddress) && !newSettings.useDhcp) || newSettings.port != m_settings.port)
        {
            connectionSettingsUpdated(ConnectionMeta(newSettings.ipAddress, newSettings.port), false);
        }
        m_settings = newSettings;
        enqueuePacket(&data[0], sizeof(data));
    }
    else
    {
        for (size_t i = 0; i < errMsgs.size(); i++)
        {
            onError(*this, "Setting " + errMsgs[i]);
        }
    }
    return ok;
}
//--------------------------------------------------------------------------------------------------
std::string MultiPcp::getConfigAsString()
{
    std::string xml;

    onXmlConfig(*this, xml);

    return xml;
}
//--------------------------------------------------------------------------------------------------
void MultiPcp::connectionEvent(bool_t isConnected)
{
    if (isConnected)
    {
        if (bootloaderMode())
        {
            Device::connectionEvent(true);
        }
        else
        {
            if (!m_connectionDataSynced)
            {
                for (const std::shared_ptr<PcpDevice>& pcpDevice : m_pcpDevices)
                {
                    pcpDevice->getSettings();
                }
                getSettings();
            }
            else
            {
                raiseConnectionEvent();
            }
        }
    }
    else
    {
        for (const std::shared_ptr<PcpDevice>& pcpDevice : m_pcpDevices)
        {
            if (pcpDevice->pcp)
            {
                m_sysPortServices.deleteSysPort(pcpDevice->pcp);
                pcpDevice->pcp.reset();
            }
        }
        
        Device::connectionEvent(false);
    }
}
//--------------------------------------------------------------------------------------------------
void MultiPcp::raiseConnectionEvent()
{
	for (const std::shared_ptr<PcpDevice>& pcpDevice : m_pcpDevices)
	{
		std::string name = "PCP-" + info.pnSnAsStr() + ":" + StringUtils::toStr(pcpDevice->id + 1);
        pcpDevice->pcp = std::make_shared<PoweredComPort>(name, *pcpDevice);
		m_sysPortServices.addSysPort(pcpDevice->pcp);
	}
	Device::connectionEvent(true);
}
//--------------------------------------------------------------------------------------------------
bool_t MultiPcp::newPacket(uint8_t command, const uint8_t* data, uint_t size)
{
    switch (static_cast<Commands>(command))
    {
	case Commands::GetSettings:
	{
        m_settings.deserialise(data, size);
       
		if (!m_connectionDataSynced)
		{
            raiseConnectionEvent();
		}
		break;
	}
	case Commands::SetSettings:
	{
		if (data[0] == 0)
		{
			onError(*this, "Settings not applied or failed to save to device");
		}
		onSettingsUpdated(*this, data[0] != 0);
		break;
	}
    case Commands::PowerStats:
    {
        uint_t channels = *data++;
        real_t voltage = Mem::getFloat32(&data);

        for (uint_t i = 0; i < m_pcpDevices.size(); i++)
        {
            if (channels & static_cast<uint_t>(1) << i)
            {
                real_t current = Mem::getFloat32(&data);
                m_pcpDevices[i]->onPowerStats(*m_pcpDevices[i], voltage, current);
            }
        }
        break;
    }
    case Commands::RouteMsg:
    {
        uint_t portId = *data++;
        uint8_t cmd = *data++;
        if (portId < m_pcpDevices.size() && cmd & 0x80)
		{
			m_pcpDevices[portId]->newPacket(cmd & 0x3f, data, size - 2);
		}
        break;
    }
    default:
        break;
    }

    return false;
}
//--------------------------------------------------------------------------------------------------
void MultiPcp::getSettings()
{
    uint8_t data = static_cast<uint8_t>(Commands::GetSettings);
    enqueuePacket(&data, sizeof(data));
}
//--------------------------------------------------------------------------------------------------
MultiPcp::Settings::Settings()
{
    defaults();
}
//--------------------------------------------------------------------------------------------------
void MultiPcp::Settings::defaults()
{
    ipAddress = Utils::ipToUint(192, 168, 1, 200);
    netmask = Utils::ipToUint(255, 255, 255, 0);
    gateway = Utils::ipToUint(192, 168, 1, 1);
    port = 33005;
    phyPortMode = PhyPortMode::Auto;
    phyMdixMode = PhyMdixMode::Normal;
    useDhcp = true;
}
//--------------------------------------------------------------------------------------------------
bool_t MultiPcp::Settings::check(std::vector<std::string>& errMsgs) const
{
    Utils::checkVar(phyPortMode, PhyPortMode::Auto, PhyPortMode::Base100TxFull, errMsgs, "phyPortMode out of range");
    Utils::checkVar(phyMdixMode, PhyMdixMode::Normal, PhyMdixMode::Auto, errMsgs, "phyMdixMode out of range");

    return errMsgs.empty();
}
//--------------------------------------------------------------------------------------------------
uint_t MultiPcp::Settings::serialise(uint8_t* buf, uint_t size) const
{
    if (size >= this->size)
    {
        uint8_t* start = buf;

        Mem::pack32Bit(&buf, ipAddress);
        Mem::pack32Bit(&buf, netmask);
        Mem::pack32Bit(&buf, gateway);
        Mem::pack16Bit(&buf, port);
        *buf++ = useDhcp;
        *buf++ = static_cast<uint8_t>(phyPortMode);
        *buf++ = static_cast<uint8_t>(phyMdixMode);

        return buf - start;
    }

    return 0;
}
//--------------------------------------------------------------------------------------------------
uint_t MultiPcp::Settings::deserialise(const uint8_t* data, uint_t size)
{
    if (size >= this->size)
    {
        const uint8_t* start = data;
        ipAddress = Mem::get32Bit(&data);
        netmask = Mem::get32Bit(&data);
        gateway = Mem::get32Bit(&data);
        port = Mem::get16Bit(&data);
        useDhcp = *data++ != 0;
        phyPortMode = static_cast<Device::PhyPortMode>(*data++);
        phyMdixMode = static_cast<Device::PhyMdixMode>(*data++);

        return data - start;
    }

    return 0;
}
//--------------------------------------------------------------------------------------------------
