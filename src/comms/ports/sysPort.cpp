//------------------------------------------ Includes ----------------------------------------------

#include "comms/ports/sysPort.h"
#include "comms/protocols/cobs.h"
#include "comms/protocols/nmea.h"
#include "comms/discovery/nmeaDiscovery.h"
#include "comms/discovery/islDeviceDiscovery.h"
#include "maths/maths.h"
#include "platform/debug.h"

using namespace IslSdk;

//--------------------------------------------------------------------------------------------------
SysPort::SysPort(const std::string& name, Type type, uint_t discoveryTimeoutMs) :
    name(name),
    type(type),
    discoveryTimeoutMs(discoveryTimeoutMs),
    id(static_cast<uint32_t>(Math::randomNum(1, Math::maxUint32))),
    m_isOpen(false),
    m_portError(false),
    m_lock(0),
    deviceCount(0),
    m_txBytesCount(0),
    m_rxBytesCount(0),
    m_badRxPacketCount(0),
    m_sdkCanClose(false)
{
}
//--------------------------------------------------------------------------------------------------
SysPort::~SysPort()
{
}
//--------------------------------------------------------------------------------------------------
void SysPort::close()
{
    if (m_isOpen)
    {
        if (m_autoDiscoverer)
        {
            m_autoDiscoverer->stop();
            m_autoDiscoverer.reset();
        }
        m_isOpen = false;
        m_portError = false;
        m_lock = 0;
        m_txBytesCount = 0;
        m_rxBytesCount = 0;
        m_badRxPacketCount = 0;
        m_sdkCanClose = false;
        debugLog("SysPort", "%s closed", name.c_str());
        onClose(*this);
    }
}
//--------------------------------------------------------------------------------------------------
void SysPort::block(uint32_t lockId)
{
    if (m_lock == 0)
    {
        m_lock = lockId;
    }
}
//--------------------------------------------------------------------------------------------------
void SysPort::unblock(uint32_t lockId)
{
    if (m_lock == lockId)
    {
        m_lock = 0;
    }
}
//--------------------------------------------------------------------------------------------------
bool_t SysPort::isBlocked()
{
    return m_lock != 0;
}
//--------------------------------------------------------------------------------------------------
void SysPort::stopDiscovery()
{
    if (m_autoDiscoverer)
    {
        m_autoDiscoverer->stop();
        m_autoDiscoverer.reset();
    }
}
//--------------------------------------------------------------------------------------------------
bool_t SysPort::isDiscovering()
{
    return m_autoDiscoverer != nullptr;
}
//--------------------------------------------------------------------------------------------------
bool_t SysPort::process()
{
    if (!m_portError && m_autoDiscoverer)
    {
        if (m_autoDiscoverer->run())
        {
            m_autoDiscoverer.reset();
        }
    }

    return m_portError;
}
//--------------------------------------------------------------------------------------------------
void SysPort::txComplete(const uint8_t* data, uint_t size)
{
    m_txBytesCount += size;
    onTxData(*this, data, size);
}
//--------------------------------------------------------------------------------------------------
const std::unique_ptr<AutoDiscovery>& SysPort::getDiscoverer()
{
    return m_autoDiscoverer;
}
//--------------------------------------------------------------------------------------------------
void SysPort::discoverIslDevices(uint16_t pid, uint16_t pn, uint16_t sn, const ConnectionMeta& meta, uint_t timeoutMs, uint_t count)
{
    if (type != Type::Net)
    {
        if (m_codec == nullptr || m_codec->type != Codec::Type::Cobs)
        {
            m_codec = std::make_unique<Cobs>(1200);
        }
    }

    if (m_autoDiscoverer == nullptr || m_autoDiscoverer->type != AutoDiscovery::Type::Isl)
    {
        m_autoDiscoverer = std::make_unique<IslDeviceDiscovery>(*this);
    }

    if (!m_isOpen)
    {
        m_sdkCanClose = open();
    }

    if (m_isOpen && m_autoDiscoverer)
    {
        IslDeviceDiscovery* discovery = reinterpret_cast<IslDeviceDiscovery*>(m_autoDiscoverer.get());
        discovery->addTask(pid, pn, sn, meta, timeoutMs, count);
    }
}
//--------------------------------------------------------------------------------------------------
void SysPort::nemaDiscovery(const ConnectionMeta& meta, uint_t timeoutMs)
{
    if (m_codec == nullptr || m_codec->type != Codec::Type::Nmea)
    {
        m_codec = std::make_unique<Nmea>(512);
    }

    if (m_autoDiscoverer == nullptr || m_autoDiscoverer->type != AutoDiscovery::Type::Nmea)
    {
        m_autoDiscoverer = std::make_unique<NmeaDiscovery>(*this);
    }

    if (!m_isOpen)
    {
        m_sdkCanClose = open();
    }

    if (m_isOpen && m_autoDiscoverer)
    {
        NmeaDiscovery* discovery = reinterpret_cast<NmeaDiscovery*>(m_autoDiscoverer.get());
        discovery->addTask(meta, timeoutMs, 1);
    }
}
//--------------------------------------------------------------------------------------------------
