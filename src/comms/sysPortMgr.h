#ifndef SYSPORTMGR_H_
#define SYSPORTMGR_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"
#include "comms/sysPortServices.h"
#include "comms/ports/sysPort.h"
#include "comms/ports/solPort.h"
#include "comms/ports/netPort.h"
#include "comms/ports/uartPort.h"
#include <list>

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    class SysPortMgr : public SysPortServices       /// This class manages all ports.
    {
        friend class Sdk;
    public:
        const std::list<SysPort::SharedPtr>& sysPortList = m_sysPortList;            ///< A read only list of ports.

        /**
        * @brief Search the classes \p m_sysPortList for a port using its id.
        * @param id The id of the port to find.
        * @return The port if found, otherwise nullptr.
        */
        const SysPort::SharedPtr findById(uint_t id) const;

        /**
        * @brief A subscribable event for newly discovered ports.
        * This event is triggered when a new port is discovered.
        * @param sysPort const SysPort::SharedPtr& The newly found port.
        */
        Signal<const SysPort::SharedPtr&> onNew;

        /**
        * @brief This function creates a serial over lan (SOL) port.
        * @param name The name of the port.
        * @param isTcp True if the network protocol should use tcp, otherwise it will be udp.
        * @param useTelnet True if the data over the network should be encoded with telnet RFC2217, otherwise the tcp/udp payload will be raw serial data.
        * @param ipAddress The ip address of the ethernet to serial converter hardware.
        * @param port The port number the ethernet to serial converter is listening on.
        * @return The newly created port.
        */
        const std::shared_ptr<SolPort> createSol(const std::string& name, bool_t isTcp, bool_t useTelnet, uint32_t ipAddress, uint16_t port);


        /**
        * @brief This function creates a UDP or TCP socket
        * @param name The name of the port.
        * @param isTcp True if the network protocol should use tcp, otherwise it will be udp.
        * @param isServer True if the port should be a server, otherwise it will be a client.
        * @param ipAddress The ip address to listen on when a TCP or UDP server. Or the ip address to connect to when a TCP client.
        * @param port The port number to listen on when a TCP or UDP server. Or the port number to connect to when a TCP client.
        * @return The newly created port.
		*/
        const std::shared_ptr<NetPort> createNetPort(const std::string& name, bool_t isTcp, bool_t isServer, uint32_t ipAddress, uint16_t port);

        /**
        * @brief Deletes a serial over lan (SOL) port.
        * This function deletes a serial over lan (SOL) port from the SDK. The port will be removed from the \p m_sysPortList
        * and raise disconnected and delete events.
        * @param sysPort The port to delete.
        */
        void deleteSolSysPort(const SysPort::SharedPtr& sysPort);

    private:
        SysPortMgr();
        void run();
        SysPort::SharedPtr getSharedPtr(SysPort& sysPort);
        uint64_t m_timer;
        std::list<SysPort::SharedPtr> m_sysPortList;
        void addSysPort(const SysPort::SharedPtr& sysPort) override;
        void deleteSysPort(const SysPort::SharedPtr& sysPort) override;
        std::list<SysPort::SharedPtr>::iterator deleteSysPort(std::list<SysPort::SharedPtr>::iterator it);
        void updateAttachedSerialPorts();
    };
}
//--------------------------------------------------------------------------------------------------
#endif
