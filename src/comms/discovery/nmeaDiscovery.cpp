//------------------------------------------ Includes ----------------------------------------------

#include "comms/discovery/nmeaDiscovery.h"
#include "platform/timeUtils.h"
#include "platform/debug.h"
#include "utils/stringUtils.h"

using namespace IslSdk;

//--------------------------------------------------------------------------------------------------
NmeaDiscovery::NmeaDiscovery(SysPort& sysPort) : AutoDiscovery(sysPort, Type::Nmea ), m_timeoutMs(0)
{
}
//--------------------------------------------------------------------------------------------------
NmeaDiscovery::~NmeaDiscovery()
{
    m_sysPort.unblock(m_sysPort.id);
}
//--------------------------------------------------------------------------------------------------
bool_t NmeaDiscovery::run()
{
    if (!m_discoveryParam.empty())
    {
        DiscoveryParam& param = m_discoveryParam.front();
        uint64_t timeMs = Time::getTimeMs();

        if (m_timeoutMs == 0)
        {
            debugLog("NmeaDiscovery", "%s discovery started", m_sysPort.name.c_str());
            m_sysPort.onDiscoveryStarted(m_sysPort, AutoDiscovery::Type::Nmea);
        }

        if (timeMs >= m_timeoutMs)
        {
            m_sysPort.unblock(m_sysPort.id);
            bool_t written = m_sysPort.write(nullptr, 0, param.meta);
            m_sysPort.block(m_sysPort.id);

            if (written)
            {
                m_timeoutMs = timeMs + param.timeoutMs;
                param.count--;
                debugLog("NmeaDiscovery", "%s discovering at %s", m_sysPort.name.c_str(), m_sysPort.type == SysPort::Type::Net ? (StringUtils::ipToStr(param.meta.ipAddress) + ":" + StringUtils::uintToStr(param.meta.port)).c_str() : StringUtils::uintToStr(param.meta.baudrate).c_str());
                m_sysPort.onDiscoveryEvent(m_sysPort, param.meta, AutoDiscovery::Type::Nmea, discoveryCount);

                if (param.count == 0)
                {
                    m_discoveryParam.pop_front();
                }
            }
            else
            {
                m_timeoutMs = timeMs + 10;
            }
        }
    }
    else if (m_isDiscovering)
    {
        if (Time::getTimeMs() >= m_timeoutMs)
        {
            m_timeoutMs = 0;
            m_isDiscovering = false;
            m_sysPort.unblock(m_sysPort.id);
            debugLog("NmeaDiscovery", "%s discovery finished", m_sysPort.name.c_str());
            m_sysPort.onDiscoveryFinished(m_sysPort, AutoDiscovery::Type::Nmea, m_discoveryCount, false);
        }
    }

    return !m_isDiscovering;
}
//--------------------------------------------------------------------------------------------------
void NmeaDiscovery::stop()
{
    m_timeoutMs = 0;
    m_discoveryParam.clear();

    if (m_isDiscovering)
    {
        m_sysPort.unblock(m_sysPort.id);
        m_sysPort.onDiscoveryFinished(m_sysPort, AutoDiscovery::Type::Nmea, m_discoveryCount, true);
    }

    m_isDiscovering = false;
}
//--------------------------------------------------------------------------------------------------
void NmeaDiscovery::addTask(const ConnectionMeta& meta, uint_t timeoutMs, uint_t count)
{
    for (DiscoveryParam& param : m_discoveryParam)
    {
        if (!param.meta.isDifferent(meta))
        {
            param.timeoutMs = timeoutMs;
            param.count += count;
            return;
        }
    }

    if (count)
    {
        if (m_discoveryParam.empty())
        {
            m_timeoutMs = 0;
            m_discoveryCount = 0;
            m_isDiscovering = true;
        }
        m_discoveryParam.emplace_back(DiscoveryParam(meta, timeoutMs, count));
    }
}
//--------------------------------------------------------------------------------------------------
