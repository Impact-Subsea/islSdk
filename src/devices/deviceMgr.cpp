//------------------------------------------ Includes ----------------------------------------------

#include "deviceMgr.h"
#include "device.h"
#include "isa500.h"
#include "isd4000.h"
#include "ism3d.h"
#include "sonar.h"
#include "platform/timeUtils.h"
#include "platform/debug.h"

using namespace IslSdk;

//--------------------------------------------------------------------------------------------------
DeviceMgr::DeviceMgr()
{
    m_timer = 0;
    m_hostCommsTimeoutMs = 500;
    m_deviceCommsTimeoutMs = 5000;
    m_txRetries = 2;
}
//--------------------------------------------------------------------------------------------------
void DeviceMgr::setCommsTimeouts(uint32_t hostCommsTimeoutMs, uint32_t deviceCommsTimeoutMs, uint_t txRetries)
{
    m_hostCommsTimeoutMs = hostCommsTimeoutMs;
    m_deviceCommsTimeoutMs = deviceCommsTimeoutMs;
    m_txRetries = txRetries;

    for (const Device::SharedPtr& device : m_deviceList)
    {
        device->setCommsTimeout(m_hostCommsTimeoutMs);
        device->m_packetResendLimit = m_txRetries;
    }
}
//--------------------------------------------------------------------------------------------------
const Device::SharedPtr DeviceMgr::findById(uint_t id) const
{
    for (const Device::SharedPtr& ptr : m_deviceList)
    {
        if (ptr->id == id)
        {
            return ptr;
        }
    }

    return Device::SharedPtr();
}
//--------------------------------------------------------------------------------------------------
const Device::SharedPtr DeviceMgr::findByPnSn(uint16_t pn, uint16_t sn) const
{
    for (const Device::SharedPtr& ptr : m_deviceList)
    {
        if (ptr->info.pn == pn && ptr->info.sn == sn)
        {
            return ptr;
        }
    }

    return Device::SharedPtr();
}
//--------------------------------------------------------------------------------------------------
const Device::SharedPtr DeviceMgr::findByAddress(uint8_t address) const
{
    for (const Device::SharedPtr& ptr : m_deviceList)
    {
        if (ptr->m_address == address)
        {
            return ptr;
        }
    }

    return Device::SharedPtr();
}
//--------------------------------------------------------------------------------------------------
void DeviceMgr::removePortFromAll(SysPort& sysPort)
{
    for (const Device::SharedPtr& device : m_deviceList)
    {
        if (device->connection && device->connection->sysPort.get() == &sysPort)
        {
            device->removePort();
        }
    }
}
//--------------------------------------------------------------------------------------------------
void DeviceMgr::remove(Device& device)
{
    std::list<Device::SharedPtr>::iterator it = m_deviceList.begin();

    while (it != m_deviceList.end())
    {
        if (it->get() == &device)
        {
            deleteDevice(it);
            break;
        }
        it++;
    }
}
//--------------------------------------------------------------------------------------------------
std::list<Device::SharedPtr>::iterator DeviceMgr::deleteDevice(std::list<Device::SharedPtr>::iterator it)
{
    Device& device = *(*it);
    debugLog("Device", "Deleting device %04u.%04u", device.info.pn, device.info.sn);
    device.disconnect();
    device.onDelete(device);
    device.removePort();

    return m_deviceList.erase(it);
}
//--------------------------------------------------------------------------------------------------
void DeviceMgr::run()
{
    uint64_t countMs = Time::getTimeMs();
    bool_t timerEvent = countMs >= m_timer;

    if (timerEvent)
    {
        m_timer = countMs + 1000;
    }

    std::list<Device::SharedPtr>::iterator it = m_deviceList.begin();

    while (it != m_deviceList.end())
    {
        if ((*it)->shouldDelete())
        {
            it = deleteDevice(it);
        }
        else
        {
            it++;
        }
    }

    std::list<Device::SharedPtr>::iterator deviceIt = m_deviceList.begin();
    std::list<Device::SharedPtr>::iterator listEndIt = m_deviceList.end();

    while (deviceIt != listEndIt)
    {
        std::list<Device::SharedPtr>::iterator currentIt = deviceIt;
        deviceIt++;
        Device::SharedPtr device = *currentIt;

        if (device->process() && device->m_isNrm)
        {
            m_deviceList.splice(m_deviceList.end(), m_deviceList, currentIt);
        }

        if (timerEvent)
        {
            device->onPacketCount(*device, device->m_packetCount.tx, device->m_packetCount.rx, device->m_packetCount.resent, device->m_packetCount.rxMissed);
        }
    }
}
//--------------------------------------------------------------------------------------------------
Device::SharedPtr DeviceMgr::createDevice(const Device::Info& deviceInfo)
{
    Device::SharedPtr device;

    switch (deviceInfo.pid)
    {
    case Device::Pid::Isa500:
        device = std::make_shared<Isa500>(deviceInfo);
        break;

    case Device::Pid::Isd4000:
        device = std::make_shared<Isd4000>(deviceInfo);
        break;

    case Device::Pid::Ism3d:
        device = std::make_shared<Ism3d>(deviceInfo);
        break;

    case Device::Pid::Sonar:
        device = std::make_shared<Sonar>(deviceInfo);
        break;

    default:
        device = std::make_shared<Device>(deviceInfo);
        break;
    }

    if (device)
    {
        debugLog("Device", "New Device %04u.%04u", deviceInfo.pn, deviceInfo.sn);
        device->setCommsTimeout(m_hostCommsTimeoutMs);
        device->m_packetResendLimit = m_txRetries;
        device->m_deviceTimeOut = m_deviceCommsTimeoutMs;
        m_deviceList.push_back(device);
    }
    return device;
}
//--------------------------------------------------------------------------------------------------
