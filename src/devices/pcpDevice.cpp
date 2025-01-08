//------------------------------------------ Includes ----------------------------------------------

#include "pcpDevice.h"
#include "platform/debug.h"
#include "maths/maths.h"
#include "utils/utils.h"

using namespace IslSdk;

//--------------------------------------------------------------------------------------------------
PcpDevice::PcpDevice(uint8_t id, Device& device, uint8_t routeCmd) : m_id(id), m_device(device), m_routeCmd(routeCmd)
{
}
//--------------------------------------------------------------------------------------------------
PcpDevice::~PcpDevice()
{
    if (pcp)
    {
        pcp->m_pcpServices = nullptr;
    }
}
//--------------------------------------------------------------------------------------------------
void PcpDevice::open()
{
    if (!m_settings.enabled)
    {
        Settings newSettings = m_settings;
        newSettings.enabled = true;
        setSettings(newSettings, false);
    }
}
//--------------------------------------------------------------------------------------------------
void PcpDevice::close()
{
    if (m_settings.enabled)
    {
        Settings newSettings = m_settings;
        newSettings.enabled = false;
        setSettings(newSettings, false);
    }
}
//--------------------------------------------------------------------------------------------------
bool_t PcpDevice::write(const uint8_t* data, uint_t size)
{
    if (data && size)
    {
        uint8_t txBuf[1000];

        while (size)
        {
            uint_t chunkSize = Math::min<uint_t>(size, sizeof(txBuf));

			txBuf[0] = m_routeCmd;
			txBuf[1] = m_id;
			txBuf[2] = static_cast<uint8_t>(Commands::Write);
			Mem::memcpy(&txBuf[3], data, chunkSize);

			if (!queuePacket(&txBuf[0], chunkSize + 3))
			{
				return false;
			}

			data += chunkSize;
			size -= chunkSize;
		}
    }
    return true;
}
//--------------------------------------------------------------------------------------------------
void PcpDevice::setSerial(uint32_t baudrate, uint8_t dataBits, Uart::Parity parity, Uart::StopBits stopBits)
{
    if (m_settings.baudrate != baudrate || m_settings.dataBits != dataBits || m_settings.parity != parity || m_settings.stopBits != stopBits)
    {
        Settings newSettings = m_settings;
        newSettings.baudrate = baudrate;
        newSettings.dataBits = dataBits;
        newSettings.parity = parity;
        newSettings.stopBits = stopBits;
        setSettings(newSettings, false);
    }
}
//--------------------------------------------------------------------------------------------------
void PcpDevice::setSerial(uint32_t baudrate)
{
    if (m_settings.baudrate != baudrate)
    {
        Settings newSettings = m_settings;
        newSettings.baudrate = baudrate;
        setSettings(newSettings, false);
    }
}
//--------------------------------------------------------------------------------------------------
bool_t PcpDevice::setPower(bool_t on)
{
    if (m_settings.powerOn != on)
    {
        Settings newSettings = m_settings;
        newSettings.powerOn = on;
        setSettings(newSettings, false);
    }

    return m_settings.powerOn;
}
//--------------------------------------------------------------------------------------------------
void PcpDevice::setMode(Uart::Mode protocol)
{
    if (m_settings.portProtocol != protocol)
    {
        Settings newSettings = m_settings;
        newSettings.portProtocol = protocol;
        setSettings(newSettings, false);
    }
}
//--------------------------------------------------------------------------------------------------
uint32_t PcpDevice::getDeviceId() const
{
    return m_device.id;
}
//--------------------------------------------------------------------------------------------------
uint_t PcpDevice::getIndex() const
{
    return id;
}
//--------------------------------------------------------------------------------------------------
ConnectionMeta PcpDevice::getDeviceConnectionMeta() const
{
    if (m_device.connection)
    {
        return m_device.connection->meta;
    }
    return ConnectionMeta(0);
}
//--------------------------------------------------------------------------------------------------
bool_t PcpDevice::setSettings(const Settings& newSettings, bool_t save)
{
    uint8_t data[Settings::size + 4];
    std::vector<std::string> errMsgs;
    bool_t ok = newSettings.check(errMsgs);

    if (ok)
    {
        data[0] = m_routeCmd;
        data[1] = m_id;
        data[2] = static_cast<uint8_t>(Commands::SetSettings);
        data[3] = static_cast<uint8_t>(save);
        newSettings.serialise(&data[4], sizeof(data) - 4);

        m_settings = newSettings;
        queuePacket(&data[0], sizeof(data));
    }
    else
    {
        for (size_t i = 0; i < errMsgs.size(); i++)
        {
            m_device.onError(m_device, "Setting " + errMsgs[i]);
        }
    }
    return ok;
}
//--------------------------------------------------------------------------------------------------
void PcpDevice::getSettings()
{
    uint8_t data[3];

    data[0] = m_routeCmd;
    data[1] = m_id;
    data[2] = static_cast<uint8_t>(Commands::GetSettings);
    queuePacket(&data[0], sizeof(data));
}
//--------------------------------------------------------------------------------------------------
void PcpDevice::newPacket(uint8_t command, const uint8_t* data, uint_t size)
{
    switch (static_cast<Commands>(command))
    {
    case Commands::GetSettings:
	{
		if (size >= Settings::size)
		{
            m_settings.deserialise(data, Settings::size);
            if (m_device.m_connectionDataSynced)
            {
                onSettingsUpdated(*this, true);
            }
		}
		break;
	}
    case Commands::SetSettings:
    {
        if (*data == 0)
        {
            m_device.onError(m_device, "Settings not applied or failed to save to device");
        }
        onSettingsUpdated(*this, *data != 0);
        break;
    }
    case Commands::Write:
    {
        break;
    }
    case Commands::Read:
    {
        if (pcp)
        {
            pcp->rxData(data, size, m_settings.baudrate);
        }
        break;
    }
    default:
        break;
    }
}
//--------------------------------------------------------------------------------------------------
bool_t PcpDevice::queuePacket(const uint8_t* data, uint_t size)
{
    return m_device.enqueuePacket(data, size);
}
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
PcpDevice::Settings::Settings()
{
    defaults();
}
//--------------------------------------------------------------------------------------------------
void PcpDevice::Settings::defaults()
{
    powerOn = false;
    enabled = false;
    portProtocol = Uart::Mode::Rs232;
    baudrate = 115200;
    dataBits = 8;
    parity = Uart::Parity::None;
    stopBits = Uart::StopBits::One;
}
//--------------------------------------------------------------------------------------------------
bool_t PcpDevice::Settings::check(std::vector<std::string>& errMsgs) const
{
    Utils::checkVar(portProtocol, Uart::Mode::Rs232, Uart::Mode::Rs485Terminated, errMsgs, "uartMode out of range");
    Utils::checkVar<uint_t>(baudrate, 300, 115200, errMsgs, "baudrate out of range");
    Utils::checkVar<uint_t>(dataBits, 5, 8, errMsgs, "dataBits out of range");
    Utils::checkVar(parity, Uart::Parity::None, Uart::Parity::Space, errMsgs, "parity out of range");
    Utils::checkVar(stopBits, Uart::StopBits::One, Uart::StopBits::Two, errMsgs, "stopBits out of range");
    
    return errMsgs.empty();
}
//--------------------------------------------------------------------------------------------------
uint_t PcpDevice::Settings::serialise(uint8_t* buf, uint_t size) const
{
    if (size >= this->size)
    {
        uint8_t* start = buf;

        *buf++ = powerOn;
        *buf++ = enabled;
        *buf++ = static_cast<uint8_t>(portProtocol);
        Mem::pack32Bit(&buf, baudrate);
        *buf++ = dataBits;
        *buf++ = static_cast<uint8_t>(parity);
        *buf++ = static_cast<uint8_t>(stopBits);
        
        return buf - start;
    }

    return 0;
}
//--------------------------------------------------------------------------------------------------
uint_t PcpDevice::Settings::deserialise(const uint8_t* data, uint_t size)
{
    if (size >= this->size)
    {
        const uint8_t* start = data;

        powerOn = *data++ != 0;
        enabled = *data++ != 0;
        portProtocol = static_cast<Uart::Mode>(*data++);
        baudrate = Mem::get32Bit(&data);
        dataBits = *data++;
        parity = static_cast<Uart::Parity>(*data++);
        stopBits = static_cast<Uart::StopBits>(*data++);

        return data - start;
    }

    return 0;
}
//--------------------------------------------------------------------------------------------------
