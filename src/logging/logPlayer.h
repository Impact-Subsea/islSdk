#ifndef LOGPLAYER_H_
#define LOGPLAYER_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"
#include "logReader.h"
#include "loggingDevice.h"
#include "types/sigSlot.h"
#include "devices/device.h"
#include "nmeaDevices/nmeaDevice.h"

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    class LogPlayer : public LogReader
    {
    public:
        LogPlayer();
        ~LogPlayer();
        bool_t open(const std::string& filename) override;
        bool_t close() override;
        void addDevice(const LoggingDevice::SharedPtr& device, const LogFile::Track& track);
        bool_t getIslDeviceTrackData(const LogFile::Track& track, Device::Info& info);
        NmeaDevice::Type getNmeaDeviceTrackData(const LogFile::Track& track);
        std::vector<uint8_t> getAppTrackData(const LogFile::Track& track);

        Signal<LogPlayer&, const LogFile::Track&> onNewTrack;

    private:
        enum class DataType { Iss360AhrsV1 = 1, Iss360SettingsV1 = 2, Iss360AhrsV2 = 3, Iss360SettingsV2 = 4, Iss360PingV1 = 11, Iss360PingV2 = 12, PacketData = 15 };
        std::vector<LoggingDevice::SharedPtr> m_devices;
        void emitRecord(const LogReader::RecordData& record) override;
        std::vector<uint8_t> convertOldFormat(const LogReader::RecordData& record);
    };
}

//--------------------------------------------------------------------------------------------------
#endif
