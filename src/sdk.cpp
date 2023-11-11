//------------------------------------------ Includes ----------------------------------------------

#include "sdk.h"
#include "utils/stringUtils.h"
#include "comms/discovery/islDeviceDiscovery.h"
#include "comms/protocols/nmea.h"
#include "platform/netSocket.h"
#include "platform/debug.h"

using namespace IslSdk;

//--------------------------------------------------------------------------------------------------
Sdk::Sdk() : m_first(true)
{
    NetSocket::initialise();
    ports.onNew.connect(m_slotNewPort);
}
//--------------------------------------------------------------------------------------------------
Sdk::~Sdk()
{
    NetSocket::deinitialise();
}
//--------------------------------------------------------------------------------------------------
void Sdk::run()
{
    if (m_first)
    {
        ports.createNetPort("NETWORK");
        m_first = false;
    }

    ports.run();
    devices.run();
}
//--------------------------------------------------------------------------------------------------
void Sdk::newPort(const SysPort::SharedPtr& sysPort)
{
    sysPort->newFrameEvent.connect(this, &Sdk::newFrameEvent);
    sysPort->onDelete.connect(m_slotPortDeleted);
}
//--------------------------------------------------------------------------------------------------
void Sdk::portDeleted(SysPort& sysPort)
{
    devices.removePortFromAll(sysPort);
    nmeaDevices.removePortFromAll(sysPort);
}
//--------------------------------------------------------------------------------------------------
void Sdk::deviceDeleted(Device& device)
{
    if (device.connection && device.connection->sysPort->type == SysPort::Type::Net)
    {
        ports.deleteSysPort(device.connection->sysPort);
    }
}
//--------------------------------------------------------------------------------------------------
void Sdk::newFrameEvent(SysPort& sysPort, const uint8_t* data, uint_t size, const ConnectionMeta& meta, Codec::Type codecType)
{
    if (codecType == Codec::Type::Nmea)
    {
        NmeaDevice::SharedPtr device = nmeaDevices.findBySysPort(sysPort);

        if (device == nullptr)
        {
            SysPort::SharedPtr sysPortPtr = ports.getSharedPtr(sysPort);

            device = nmeaDevices.createDevice(sysPortPtr, data, size);
            if (device)
            {
                nmeaDevices.onNew(device, sysPortPtr, meta);

                if (sysPort.m_autoDiscoverer)
                {
                    sysPort.m_autoDiscoverer->hasDiscovered();
                    sysPort.m_autoDiscoverer->stop();
                }
            }
        }

        if (device)
        {
            device->newPacketEvent(data, size);
        }
    }
    else
    {
        IslHdlcPacket packet;

        if (packet.fromFrame(data, size))
        {
            if (packet.header.type == IslHdlcPacket::FrameType::U && packet.header.uCode == IslHdlcPacket::UframeCode::Discover && !packet.header.cr && packet.size >= Device::Info::size)
            {
                Device::Info deviceInfo = Device::Info(packet.payload);
                deviceInfo.inUse = packet.header.address != 0;

                Device::SharedPtr device = devices.findByPnSn(deviceInfo.pn, deviceInfo.sn);
                SysPort::SharedPtr sysPortPtr = ports.getSharedPtr(sysPort);
                bool_t newDevice = device == nullptr;

                if (newDevice)
                {
                    device = devices.createDevice(deviceInfo);
                    device->onDelete.connect(m_slotDeviceDeleted);
                }
                else if (device->m_address == packet.header.address)
                {
                    deviceInfo.inUse = false;
                }

                if (sysPort.type == SysPort::Type::Net)
                {
                    if (device->connection && device->connection->sysPort->type == SysPort::Type::Net)
                    {
                        sysPortPtr = device->connection->sysPort;
                    }
                    else
                    {
                        sysPortPtr = ports.createNetPort("NET " + device->info.pnSnAsStr());
                        sysPortPtr->m_sdkCanClose = sysPortPtr->open();
                    }
                }

                device->addOrUpdatePort(sysPortPtr, meta);

                if (newDevice)
                {
                    devices.onNew(device, sysPortPtr, meta);
                }
                else
                {
                    if (device->info.isDifferent(deviceInfo))
                    {
                        device->m_info = deviceInfo;
                        device->onInfoChanged(*device, deviceInfo);
                    }
                    device->connect();
                }

                if (sysPort.m_autoDiscoverer)
                {
                    sysPort.m_autoDiscoverer->hasDiscovered();

                    if (sysPort.m_autoDiscoverer->type == AutoDiscovery::Type::Isl)
                    {
                        (reinterpret_cast<IslDeviceDiscovery*>(sysPort.m_autoDiscoverer.get()))->removeTask(0xffff, 0xffff, 0xffff);
                        (reinterpret_cast<IslDeviceDiscovery*>(sysPort.m_autoDiscoverer.get()))->removeTask(static_cast<uint16_t>(deviceInfo.pid), deviceInfo.pn, deviceInfo.sn);
                    }
                }
            }
            else
            {
                const Device::SharedPtr& device = devices.findByAddress(packet.header.address);

                if (device)
                {
                    device->processPacket(packet);
                }
            }
        }
        else
        {
            sysPort.m_badRxPacketCount++;
        }
    }
}
//--------------------------------------------------------------------------------------------------
