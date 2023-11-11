//------------------------------------------ Includes ----------------------------------------------

#include "nmeaDevice.h"
#include "comms/protocols/nmea.h"
#include "utils/stringUtils.h"

using namespace IslSdk;

//--------------------------------------------------------------------------------------------------
NmeaDevice::NmeaDevice(Type type) : type(type)
{
}
//--------------------------------------------------------------------------------------------------
NmeaDevice::NmeaDevice(const SysPort::SharedPtr& sysPort, Type type) : m_sysPort(sysPort), type(type)
{
    sysPort->deviceCount++;
}
//--------------------------------------------------------------------------------------------------
NmeaDevice::~NmeaDevice()
{
    if (m_sysPort)
    {
        m_sysPort->deviceCount--;
    }
}
//--------------------------------------------------------------------------------------------------
void NmeaDevice::newPacketEvent(const uint8_t* data, uint_t size)
{
    if (Nmea::checkCrc(data, size))
    {
        std::string sentence = StringUtils::toStr(data, size);

        if (newSentence(sentence))
        {
            log(data, size, static_cast<uint8_t>(LoggingDataType::packetData));
        }
    }
}
//---------------------------------------------------------------------------------------------------
void NmeaDevice::removePort(const SysPort& sysPort)
{
    if (&sysPort == m_sysPort.get())
    {
        m_sysPort->deviceCount--;
        m_sysPort.reset();
    }
}
//---------------------------------------------------------------------------------------------------
