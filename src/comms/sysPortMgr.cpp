//------------------------------------------ Includes ----------------------------------------------

#include "comms/sysPortMgr.h"
#include "comms/ports/uartPort.h"
#include "platform/timeUtils.h"
#include "platform/debug.h"
#include "utils/stringUtils.h"

using namespace IslSdk;

//--------------------------------------------------------------------------------------------------
SysPortMgr::SysPortMgr() : m_timer(0)
{
}
//--------------------------------------------------------------------------------------------------
const SysPort::SharedPtr SysPortMgr::findById(uint_t id) const
{
    for (const SysPort::SharedPtr& sysPort : m_sysPortList)
    {
        if (sysPort->id == id)
        {
            return sysPort;
        }
    }

    return SysPort::SharedPtr();
}
//--------------------------------------------------------------------------------------------------
SysPort::SharedPtr SysPortMgr::getSharedPtr(SysPort& sysPort)
{
    for (SysPort::SharedPtr& sysPortPtr : m_sysPortList)
    {
        if (&sysPort == sysPortPtr.get())
        {
            return sysPortPtr;
        }
    }
    return SysPort::SharedPtr();
}
//--------------------------------------------------------------------------------------------------
const std::shared_ptr<NetPort> SysPortMgr::createNetPort(const std::string& name)
{
    const std::shared_ptr<NetPort> ptr = std::make_shared<NetPort>(name, false, false, 0xffffffff, 0);
    m_sysPortList.push_back(ptr);
    debugLog("SysPort", "New network port %s", name.c_str());
    onNew(m_sysPortList.back());

    return ptr;
}
//--------------------------------------------------------------------------------------------------
const std::shared_ptr<SolPort> SysPortMgr::createSol(const std::string& name, bool_t isTcp, bool_t useTelnet, uint32_t ipAddress, uint16_t port)
{
    std::shared_ptr<SolPort> ptr;

    if (name.empty())
    {
        std::string solName = "SOL: " + StringUtils::ipToStr(ipAddress) + ":" + StringUtils::uintToStr(port);
        ptr = std::make_shared<SolPort>(solName, isTcp, useTelnet, ipAddress, port);
    }
    else
    {
        ptr = std::make_shared<SolPort>(name, isTcp, useTelnet, ipAddress, port);
    }
    m_sysPortList.push_back(ptr);
    debugLog("SysPort", "New SOL port %s", name.c_str());
    onNew(m_sysPortList.back());

    return ptr;
}
//--------------------------------------------------------------------------------------------------
void SysPortMgr::deleteSolSysPort(const SysPort::SharedPtr& sysPort)
{
    if (sysPort->type == SysPort::Type::Sol)
    {
        deleteSysPort(sysPort);
    }
}
//--------------------------------------------------------------------------------------------------
void SysPortMgr::deleteSysPort(const SysPort::SharedPtr& sysPort)
{
    std::list<SysPort::SharedPtr>::iterator it = m_sysPortList.begin();

    while (it != m_sysPortList.end())
    {
        if (*it == sysPort)
        {
            deleteSysPort(it);
            break;
        }
        it++;
    }
}
//--------------------------------------------------------------------------------------------------
std::list<SysPort::SharedPtr>::iterator SysPortMgr::deleteSysPort(std::list<SysPort::SharedPtr>::iterator it)
{
    SysPort& sysPort = *(it->get());
    sysPort.close();
    debugLog("SysPort", "Deleting port %s", sysPort.name.c_str());
    sysPort.onDelete(sysPort);

    return m_sysPortList.erase(it);
}
//--------------------------------------------------------------------------------------------------
void SysPortMgr::updateAttachedSerialPorts()
{
    std::vector<std::string> nameList = Uart::getNames();
    std::list<SysPort::SharedPtr>::iterator it = m_sysPortList.begin();

    while (it != m_sysPortList.end())
    {
        SysPort* sysPort = it->get();
        if (sysPort->type == SysPort::Type::Serial)
        {
            bool_t inList = false;
            for (size_t i = 0; i < nameList.size(); i++)
            {
                if (sysPort->name == nameList[i])
                {
                    inList = true;
                    break;
                }
            }

            if (!inList)
            {
                it = deleteSysPort(it);
                continue;
            }
        }
        it++;
    }

    for (size_t i = 0; i < nameList.size(); i++)
    {
        bool_t inList = false;
        for (const SysPort::SharedPtr& sysPort : m_sysPortList)
        {
            if (sysPort->name == nameList[i])
            {
                inList = true;
                break;
            }
        }

        if (!inList)
        {
            m_sysPortList.emplace_back(new UartPort(nameList[i]));
            debugLog("SysPort", "New Serial port %s", (m_sysPortList.back())->name.c_str());
            onNew(m_sysPortList.back());
        }
    }
}
//--------------------------------------------------------------------------------------------------
void SysPortMgr::run()
{
    uint64_t countMs = Time::getTimeMs();
    bool_t timerEvent = countMs >= m_timer;

    if (timerEvent)
    {
        m_timer = countMs + 1000;
        updateAttachedSerialPorts();
    }

    std::list<SysPort::SharedPtr>::iterator it = m_sysPortList.begin();

    while (it != m_sysPortList.end())
    {
        SysPort* sysPort = (*it).get();
        if (sysPort->isOpen)
        {
            if (sysPort->process())
            {
                sysPort->onError(*sysPort, "Port error, deleting port");
                it = deleteSysPort(it);
                continue;
            }

            if (timerEvent)
            {
                sysPort->onPortStats(*sysPort, sysPort->m_txBytesCount, sysPort->m_rxBytesCount, sysPort->m_badRxPacketCount);
                sysPort->m_rxBytesCount = 0;
                sysPort->m_txBytesCount = 0;
                sysPort->m_badRxPacketCount = 0;
            }

            if (sysPort->m_sdkCanClose && (sysPort->deviceCount == 0) && !sysPort->m_autoDiscoverer)
            {
                sysPort->close();
            }
        }
        else if (sysPort->m_sdkCanClose && (sysPort->deviceCount == 0))
        {
            it = deleteSysPort(it);
            continue;
        }
        it++;
    }
}
//--------------------------------------------------------------------------------------------------
