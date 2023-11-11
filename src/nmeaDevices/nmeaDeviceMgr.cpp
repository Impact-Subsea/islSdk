//------------------------------------------ Includes ----------------------------------------------

#include "nmeaDeviceMgr.h"
#include "nmeaDevice.h"
#include "gpsDevice.h"
#include "comms/protocols/nmea.h"
#include "utils/stringUtils.h"
#include "platform/debug.h"

using namespace IslSdk;

//--------------------------------------------------------------------------------------------------
NmeaDeviceMgr::NmeaDeviceMgr()
{
}
//--------------------------------------------------------------------------------------------------
NmeaDevice::SharedPtr NmeaDeviceMgr::createDevice(const SysPort::SharedPtr& sysPortPtr, const uint8_t* data, uint_t size)
{
    NmeaDevice::SharedPtr ptr;

    if (Nmea::checkCrc(data, size))
    {
        std::string sentence = StringUtils::toStr(data, size);
        GpsDevice::SentenceType type = GpsDevice::getSentenceType(sentence);

        if (type != GpsDevice::SentenceType::Unsupported)
        {
            ptr = std::make_shared<GpsDevice>(sysPortPtr);
        }
    }

    if (ptr)
    {
        m_deviceList.push_back(ptr);
    }

    return ptr;
}
//--------------------------------------------------------------------------------------------------
void NmeaDeviceMgr::removePortFromAll(const SysPort& sysPort)
{
    for (const NmeaDevice::SharedPtr& device : m_deviceList)
    {
        device->removePort(sysPort);
    }
}
//--------------------------------------------------------------------------------------------------
NmeaDevice::SharedPtr NmeaDeviceMgr::findBySysPort(const SysPort& sysPort)
{
    for (const NmeaDevice::SharedPtr& device : m_deviceList)
    {
        if (device->sysPort.get() == &sysPort)
        {
            return device;
        }
    }
    return NmeaDevice::SharedPtr();
}
//--------------------------------------------------------------------------------------------------
void NmeaDeviceMgr::remove(NmeaDevice& device)
{
    std::list<NmeaDevice::SharedPtr>::iterator it = m_deviceList.begin();

    while (it != m_deviceList.end())
    {
        if (it->get() == &device)
        {
            device.onDelete(device);
            m_deviceList.erase(it);
            break;
        }
        it++;
    }
}
//--------------------------------------------------------------------------------------------------
