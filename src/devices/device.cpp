//------------------------------------------ Includes ----------------------------------------------

#include "device.h"
#include "comms/discovery/autoDiscovery.h"
#include "platform/mem.h"
#include "platform/debug.h"
#include "maths/maths.h"
#include "platform/timeUtils.h"
#include "logging/logPlayer.h"
#include "utils/stringUtils.h"

using namespace IslSdk;

//--------------------------------------------------------------------------------------------------
Device::Info::Info() : pid(Device::Pid::Unknown), pn(0), sn(0), config(0), mode(0), status(0),
firmwareBuildNum(0), firmwareVersionBcd(0), inUse(false)
{
}
//--------------------------------------------------------------------------------------------------
Device::Info::Info(const uint8_t* data)
{
    fromBuf(data);
}
//--------------------------------------------------------------------------------------------------
void Device::Info::fromBuf(const uint8_t* data)
{
    pid = static_cast<Pid>(Mem::get16Bit(&data));
    pn = Mem::get16Bit(&data);
    sn = Mem::get16Bit(&data);
    config = *data++;
    mode = *data++;
    status = Mem::get16Bit(&data);
    firmwareBuildNum = Mem::get16Bit(&data);
    firmwareVersionBcd = Mem::get16Bit(&data);
    inUse = false;
}
//--------------------------------------------------------------------------------------------------
bool_t Device::Info::isDifferent(const Info& d) const
{
    return (pn != d.pn || sn != d.sn || pid != d.pid || config != d.config || mode != d.mode || status != d.status || firmwareBuildNum != d.firmwareBuildNum ||
        firmwareVersionBcd != d.firmwareVersionBcd || inUse != d.inUse);
}
//--------------------------------------------------------------------------------------------------
std::string Device::Info::pnSnAsStr() const
{
    return StringUtils::pnSnToStr(pn, sn);
}
//--------------------------------------------------------------------------------------------------
std::string Device::Info::name() const
{
	return StringUtils::pidToStr(pid);
}
//--------------------------------------------------------------------------------------------------
uint8_t Device::CustomStr::packStr(uint8_t* ptr) const
{
    bool_t esc = false;
    uint_t len = 0;

    for (uint_t i = 0; i < str.size() && len < size; i++)
    {
        uint8_t c = static_cast<uint8_t>(str[i]);
        if (esc)
        {
            if (c == 'r') c = 13;
            else if (c == 'n') c = 10;
            else if (c == 't') c = 9;          
            else if ((c == 'x' || c == 'X') && (str.size() - i > 2))
			{
                bool_t e = false;
                i++;
                c = static_cast<uint8_t>(StringUtils::hexToUint(str, i, 2, e));
                if (!e) i--;
			}
            else if (c == '0' && (str.size() - i > 3) && (str[i+1] == 'x' || str[i+1] == 'X'))
            {
                bool_t e = false;
                i += 2;
                c = static_cast<uint8_t>(StringUtils::hexToUint(str, i, 2, e));
                if (!e) i--;
            }
            esc = false;
        }
        else if (c == '\\')
        {
            esc = true;
            continue;
        }
   
        ptr[len++] = c;
    }

    for (uint_t i = len; i < size; i++)
    {
        ptr[i] = 0;
    }
    return static_cast<uint8_t>(len);
}
//--------------------------------------------------------------------------------------------------
void Device::CustomStr::unPackStr(const uint8_t* data, uint_t size)
{
    str.clear();

    for (uint_t i = 0; i < size; i++)
    {
        if (std::isprint(data[i]))
        {
            str += data[i];
        }
        else
        {
            if (data[i] == 10)
			{
				str += "\\n";
			}
			else if (data[i] == 13)
			{
				str += "\\r";
			}
			else if (data[i] == 9)
			{
				str += "\\t";
			}
			else
			{
				str += "\\0x" + StringUtils::toHexStr(static_cast<uint_t>(data[i]), 1);
			}
        }
    }
}
//--------------------------------------------------------------------------------------------------
Device::Device(const Device::Info& info) : IslHdlc(), m_info(info)
{
    m_connectionDataSynced = false;
    m_reconnectCount = 0;
    m_resetting = false;
    m_searchTimeoutMs = 1000;
    m_searchCount = 30;
    m_packetResendLimit = 2;
    m_deviceTimeOut = 0;
    m_deleteTimer = Time::getTimeMs() + deleteAfterTime;          // Delete this device if not connected within deleteAfterTime milliseconds
    m_epochUs = Time::getTimeMs() * 1000;
    m_waitingForXmlConfig = false;
}
//--------------------------------------------------------------------------------------------------
Device::~Device()
{
}
//---------------------------------------------------------------------------------------------------
void Device::connect()
{
    if (!m_connected)
    {
        if (m_connection)
        {
            IslHdlc::connect(m_info.pn, m_info.sn, m_deviceTimeOut);
            m_deleteTimer = Time::getTimeMs() + 1000 + m_timeoutMs;
        }
    }
}
//---------------------------------------------------------------------------------------------------
void Device::setRediscoveryTimeouts(uint_t searchTimeoutMs, uint_t searchCount)
{
    m_searchTimeoutMs = searchTimeoutMs;
    m_searchCount = searchCount;
}
//---------------------------------------------------------------------------------------------------
void Device::setCommsRetries(uint_t retries)
{
    m_packetResendLimit = retries;
}
//---------------------------------------------------------------------------------------------------
void Device::reset()
{
    m_resetting = true;
    const uint8_t payload[1] = { static_cast<uint8_t>(Commands::Reset) };
    sendPacket(&payload[0], sizeof(payload));
}
//---------------------------------------------------------------------------------------------------
bool_t Device::isConnected()
{
    return m_connected && (m_connectionDataSynced || bootloaderMode());
}

//---------------------------------------------------------------------------------------------------
void Device::connectionEvent(bool_t connected)
{
    if (connected)
    {
        debugLog("Device", "Device %s connected", info.pnSnAsStr().c_str());
        m_connectionDataSynced = !bootloaderMode();
        m_info.inUse = false;
        m_resetting = false;
        onConnect(*this);
        m_reconnectCount++;
        
    }
    else
    {
        debugLog("Device", "Device %s disconnected", info.pnSnAsStr().c_str());
        m_connectionDataSynced = false;
        onDisconnect(*this);
    }
}
//--------------------------------------------------------------------------------------------------
bool_t Device::enqueuePacket(const uint8_t* data, uint_t size)
{
    if (m_connected)
    {
        if (!bootloaderMode())
        {
            return sendPacket(data, size);
        }
    }
    else
    {
        onError(*this, "Device must be connected before commands are sent");
    }
    return false;
}
//---------------------------------------------------------------------------------------------------
bool_t Device::sendPacket(const uint8_t* data, uint_t size)
{
    if (m_connection)
    {
        if (m_connection->sysPort->isOpen)
        {
            send(data, size);
            return true;
        }
    }
    return false;
}
//---------------------------------------------------------------------------------------------------
void Device::connectionSettingsUpdated(const ConnectionMeta& meta, bool_t isHalfDuplex)
{
    if (m_connection)
    {
        m_pendingMeta = std::make_unique<ConnectionMeta>(meta);

        if (!m_isNrm && isHalfDuplex)
        {
            setNrmMode(true);
        }
    }
}
//---------------------------------------------------------------------------------------------------
bool_t Device::startLogging()
{
    LoggingDevice::startLogging();

    uint8_t buf[9];
    buf[0] = 1;
    Mem::pack64Bit(&buf[1], m_epochUs);
    return log(&buf[0], sizeof(buf), static_cast<uint8_t>(LoggingDataType::LogData), false);
}
//---------------------------------------------------------------------------------------------------
bool_t Device::shouldDelete() const
{
    bool_t deleteDevice = (m_connection && !m_connection->sysPort->isOpen) || (m_connection == nullptr) || (m_deleteTimer && (Time::getTimeMs() >= m_deleteTimer));

    return deleteDevice;
}
//---------------------------------------------------------------------------------------------------
void Device::addOrUpdatePort(const SysPort::SharedPtr& sysPort, const ConnectionMeta& meta)
{
    if (m_connection && m_connection->sysPort.get() == sysPort.get())
    {
        updateConnection(meta);
    }
    else
    {
        if (m_connection)
        {
            removePort();
        }
        m_connection = std::make_unique<Connection>(sysPort, meta);
        sysPort->deviceCount++;
        onPortAdded(*this, *sysPort, meta);
    }
}
//---------------------------------------------------------------------------------------------------
void Device::removePort()
{
    if (m_connection)
    {
        m_connection->sysPort->unblock(id);
        m_connection->sysPort->deviceCount--;
        onPortRemoved(*this, *m_connection->sysPort);
        m_connection.reset();
    }
}
//---------------------------------------------------------------------------------------------------
void Device::updateConnection(const ConnectionMeta& meta)
{
    if (m_connection->meta.baudrate != meta.baudrate || m_connection->meta.ipAddress != meta.ipAddress || m_connection->meta.port != meta.port)
    {
        m_connection->meta = meta;
        onPortChanged(*this, *m_connection->sysPort, meta);
    }
}
//---------------------------------------------------------------------------------------------------
uint_t Device::reDiscover(uint_t timeoutMs, uint_t count)
{
    if (m_connection && count)
    {
        if (m_isNrm && m_connection->sysPort->deviceCount > 1)
        {
            count = 1;
        }
        m_connection->sysPort->discoverIslDevices(static_cast<uint16_t>(m_info.pid), m_info.pn, m_info.sn, m_connection->meta, timeoutMs, count);
    }
    return timeoutMs * count;
}
//---------------------------------------------------------------------------------------------------
void Device::hdlcConnectionEvent(bool_t connected)
{
    if (connected)
    {
        m_epochUs = Time::getTimeMs() * 1000;
        m_deleteTimer = 0;
        uint8_t buf[9];
        buf[0] = 1;
        Mem::pack64Bit(&buf[1], m_epochUs);
        log(&buf[0], sizeof(buf), static_cast<uint8_t>(LoggingDataType::LogData), false);
    }
    connectionEvent(connected);
}
//---------------------------------------------------------------------------------------------------
bool_t Device::timeoutEvent()
{
    bool_t tryAgain = true;

    if (m_resetting)
    {
        disconnect();
        debugLog("Device", "Device %s being rediscovered after SDK issuing reset", info.pnSnAsStr().c_str());
        m_deleteTimer = Time::getTimeMs() + reDiscover(m_searchTimeoutMs, 30);
        m_resetting = false;
    }

    if (m_timeoutCount >= m_packetResendLimit)
    {
        debugLog("Device", "Comms lost");
        m_deleteTimer = Time::getTimeMs() + reDiscover(m_searchTimeoutMs, m_searchCount);
        tryAgain = false;
    }
    else
    {
        debugLog("Device", "Comms timeout");

        if (m_connection && m_pendingMeta)
        {
            updateConnection(*m_pendingMeta);
            m_pendingMeta.reset();
            debugLog("Device", "Updating bauadrate/ip address and retrying");
        }
    }

    onCommsTimeout(*this, !tryAgain);
    return tryAgain;
}
//---------------------------------------------------------------------------------------------------
void Device::newPacketEvent(const uint8_t* data, uint_t size)
{
    if (size >= 1)
    {
        bool_t isReply = (*data & 0x80) != 0;
        Commands command = static_cast<Commands>(*data & 0x7f);
        data++;
        size--;

        if (isReply)
        {
            switch (command)
            {
            case Commands::Descriptor:
            {
                if (size >= Device::Info::size)
                {
                    Device::Info deviceInfo = Device::Info(data);

                    if (deviceInfo.isDifferent(m_info))
                    {
                        m_info = deviceInfo;
                        onInfoChanged(*this, m_info);
                    }
                }
                break;
            }
            
            default:
                break;
            }

            if (!bootloaderMode())
            {
                if (newPacket(static_cast<uint8_t>(command), data, size))
                {
                    log(--data, size + 1, static_cast<uint8_t>(LoggingDataType::packetData));
                }
            }
            
        }
    }
}
//--------------------------------------------------------------------------------------------------
LoggingDevice::Type Device::getTrackData(std::vector<uint8_t>& buf)
{
    uint_t size = buf.size();
    buf.resize(buf.size() + 14);
    uint8_t* ptr = &buf[size];

    Mem::pack16Bit(&ptr, static_cast<uint16_t>(m_info.pid));
    Mem::pack16Bit(&ptr, m_info.pn);
    Mem::pack16Bit(&ptr, m_info.sn);
    *ptr++ = m_info.config;
    *ptr++ = m_info.mode;
    Mem::pack16Bit(&ptr, m_info.status);
    Mem::pack16Bit(&ptr, m_info.firmwareBuildNum);
    Mem::pack16Bit(&ptr, m_info.firmwareVersionBcd);

    return LoggingDevice::Type::Isl;
}
//--------------------------------------------------------------------------------------------------
void Device::logData(uint8_t dataType, const std::vector<uint8_t> data)
{
    if (dataType == static_cast<uint8_t>(LoggingDataType::LogData) && data.size() >= 9 && data[0] == 1)
    {
        m_epochUs = Mem::get64Bit(&data[1]);
    }
}
//--------------------------------------------------------------------------------------------------

