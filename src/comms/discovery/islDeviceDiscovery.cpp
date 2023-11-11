//------------------------------------------ Includes ----------------------------------------------

#include "comms/discovery/islDeviceDiscovery.h"
#include "comms/islHdlc.h"
#include "comms/ports/sysPort.h"
#include "platform/timeUtils.h"
#include "platform/debug.h"
#include "utils/stringUtils.h"

using namespace IslSdk;

//--------------------------------------------------------------------------------------------------
IslDeviceDiscovery::IslDeviceDiscovery(SysPort& sysPort) : AutoDiscovery(sysPort, Type::Isl ), m_timeoutMs(0)
{
}
//--------------------------------------------------------------------------------------------------
IslDeviceDiscovery::~IslDeviceDiscovery()
{
    m_sysPort.unblock(m_sysPort.id);
}
//--------------------------------------------------------------------------------------------------
bool_t IslDeviceDiscovery::run()
{
    if (!m_discoveryParam.empty())
    {
        DiscoveryParam& param = m_discoveryParam.front();
        uint64_t timeMs = Time::getTimeMs();

        if (m_timeoutMs == 0)
        {
            debugLog("IslDeviceDiscovery", "%s discovery started", m_sysPort.name.c_str());
            m_sysPort.onDiscoveryStarted(m_sysPort, AutoDiscovery::Type::Isl);
        }

        if (timeMs >= m_timeoutMs)
        {
            const std::vector<uint8_t> buf = IslHdlc::BuildDiscovery(param.pid, param.pn, param.sn);
            m_sysPort.unblock(m_sysPort.id);
            bool_t written = m_sysPort.write(&buf[0], buf.size(), param.meta);
            m_sysPort.block(m_sysPort.id);

            if (written)
            {
                m_timeoutMs = timeMs + param.timeoutMs;
                param.count--;
                debugLog("IslDeviceDiscovery", "%s discovering at %s", m_sysPort.name.c_str(), m_sysPort.type == SysPort::Type::Net ? (StringUtils::ipToStr(param.meta.ipAddress) + ":" + StringUtils::uintToStr(param.meta.port)).c_str() : StringUtils::uintToStr(param.meta.baudrate).c_str());
                m_sysPort.onDiscoveryEvent(m_sysPort, param.meta, AutoDiscovery::Type::Isl, discoveryCount);

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
            debugLog("IslDeviceDiscovery", "%s discovery finished", m_sysPort.name.c_str());
            m_sysPort.onDiscoveryFinished(m_sysPort, AutoDiscovery::Type::Isl, m_discoveryCount, false);
        }
    }

    return !m_isDiscovering;
}
//--------------------------------------------------------------------------------------------------
void IslDeviceDiscovery::stop()
{
    m_timeoutMs = 0;
    m_discoveryParam.clear();

    if (m_isDiscovering)
    {
        m_sysPort.unblock(m_sysPort.id);
        m_sysPort.onDiscoveryFinished(m_sysPort, AutoDiscovery::Type::Isl, m_discoveryCount, true);
    }

    m_isDiscovering = false;
}
//--------------------------------------------------------------------------------------------------
void IslDeviceDiscovery::addTask(uint16_t pid, uint16_t pn, uint16_t sn, const ConnectionMeta& meta, uint_t timeoutMs, uint_t count)
{
    for (DiscoveryParam& param : m_discoveryParam)
    {
        if (param.pid == pid && param.pn == pn && param.sn == sn && !param.meta.isDifferent(meta))
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
        m_discoveryParam.emplace_back(DiscoveryParam(pid, pn, sn, meta, timeoutMs, count));
    }
}
//--------------------------------------------------------------------------------------------------
void IslDeviceDiscovery::removeTask(uint16_t pid, uint16_t pn, uint16_t sn)
{
    std::list<DiscoveryParam>::iterator it = m_discoveryParam.begin();

    while (it != m_discoveryParam.end())
    {
        if (it->pid == pid && it->pn == pn && it->sn == sn)
        {
            if (it == m_discoveryParam.begin())
            {
                m_timeoutMs = Time::getTimeMs();
            }
            it = m_discoveryParam.erase(it);
            continue;
        }
        it++;
    }
}
//--------------------------------------------------------------------------------------------------
