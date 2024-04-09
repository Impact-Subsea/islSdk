//------------------------------------------ Includes ----------------------------------------------

#include "logPlayer.h"
#include "platform/mem.h"

using namespace IslSdk;

//--------------------------------------------------------------------------------------------------
LogPlayer::LogPlayer()
{
}
//--------------------------------------------------------------------------------------------------
LogPlayer::~LogPlayer()
{
}
//--------------------------------------------------------------------------------------------------
bool_t LogPlayer::open(const std::string& filename)
{
    m_devices.clear();

    if (LogReader::open(filename))
    {
        uint_t maxTrackId = 0;

        for (const LogFile::Track& track : m_file.tracks)
        {
            if (track.id > maxTrackId)
            {
                maxTrackId = track.id;
            }
        }
        m_devices.resize(maxTrackId + 1);

        for (const LogFile::Track& track : m_file.tracks)
        {
            onNewTrack(*this, track);
        }

        return true;
    }
    return false;
}
//--------------------------------------------------------------------------------------------------
bool_t LogPlayer::close()
{
    m_devices.clear();
    return LogReader::close();
}
//--------------------------------------------------------------------------------------------------
void LogPlayer::addDevice(const LoggingDevice::SharedPtr& device, const LogFile::Track& track)
{
    if (device)
    {
        m_devices[track.id] = device;
    }
}
//--------------------------------------------------------------------------------------------------
bool_t LogPlayer::getIslDeviceTrackData(const LogFile::Track& track, Device::Info& info)
{
    if (track.dataType == static_cast<uint8_t>(LoggingDevice::Type::Isl) && track.data.size() >= 14)
    {
        info.fromBuf(&track.data[0]);
        return true;
    }
    else if (track.dataType == static_cast<uint8_t>(LoggingDevice::Type::Unknown) && track.data.size() == 11)
    {
        info.pid = Device::Pid::Sonar;
        info.pn = Mem::get16Bit(&track.data[7]);
        info.sn = Mem::get16Bit(&track.data[5]);
        info.config = 0;
        info.mode = 0;
        info.status = 0;
        info.firmwareBuildNum = 0;
        info.firmwareVersionBcd = 0;
        info.inUse = false;
        return true;
    }
    return false;
}
//--------------------------------------------------------------------------------------------------
NmeaDevice::Type LogPlayer::getNmeaDeviceTrackData(const LogFile::Track& track)
{
    if (track.dataType == static_cast<uint8_t>(LoggingDevice::Type::Nmea) && track.data.size() >= 1)
    {
        return static_cast<NmeaDevice::Type>(track.data[0]);
    }

    return NmeaDevice::Type::Unknown;
}
//--------------------------------------------------------------------------------------------------
std::vector<uint8_t> LogPlayer::getAppTrackData(const LogFile::Track& track)
{
    std::vector<uint8_t> data;

    if (track.dataType == static_cast<uint8_t>(LoggingDevice::Type::Isl) && track.data.size() >= 14)
    {
        data.insert(data.begin(), track.data.begin() + 14, track.data.end());
    }
    else if (track.dataType == static_cast<uint8_t>(LoggingDevice::Type::Nmea) && track.data.size() >= 1)
    {
        data.insert(data.begin(), track.data.begin() + 1, track.data.end());
    }
    else if (track.dataType == static_cast<uint8_t>(LoggingDevice::Type::Unknown) && track.data.size() == 11)
    {
        data.insert(data.begin(), 8, 0);
        data[0] = 4;
    }

    return data;
}
//--------------------------------------------------------------------------------------------------
void LogPlayer::emitRecord(const LogReader::RecordData& record)
{
    std::shared_ptr<LoggingDevice> device = m_devices[record.trackId];

    if (device)
    {
        if (record.recordType == LogFile::RecordHeader::Type::Data)
        {
            if (record.dataType == static_cast<uint8_t>(Device::LoggingDataType::packetData))
            {
                device->newPacketEvent(&record.data[0], record.data.size());
            }
            else
            {
                device->logData(record.dataType, record.data);
            }
        }
        else if (record.recordType == LogFile::RecordHeader::Type::Meta)
        {
            std::vector<uint8_t> data = convertOldFormat(record);
            if (data.size())
            {
                device->newPacketEvent(&data[0], data.size());
            }
        }
    }

    LogReader::emitRecord(record);
}
//--------------------------------------------------------------------------------------------------
std::vector<uint8_t> LogPlayer::convertOldFormat(const LogReader::RecordData& record)
{
    std::vector<uint8_t> data;
    DataType dataTye = static_cast<DataType>(record.dataType);

    switch (dataTye)
    {
    case DataType::Iss360AhrsV1:
    case DataType::Iss360AhrsV2:
    {
        if (record.data.size() <= 24)
        {
            data.resize(29);
            data[0] = 0x8a;
            Mem::pack32Bit(&data[1], 1);
            Mem::memcpy(&data[5], &record.data[0], record.data.size());
        }
        break;
    }
    case DataType::Iss360SettingsV1:
    {
        if (record.data.size() == 90)
        {
            data.resize(180);
            data[0] = 0x8c;
            Mem::memcpy(&data[1], &record.data[0], 6);      // macAddress
            data[7] = record.data[10];                      // uartMode
            Mem::memcpy(&data[8], &record.data[6], 4);      // baudrate
            Mem::memcpy(&data[12], &record.data[11], 12);   // ipAddress, netmask, gateway
            Mem::pack32Bit(&data[24], 33005);               // port
            data[26] = record.data[23];                     // phyPortMode
            data[27] = record.data[24];                     // phyMdixMode
            data[28] = record.data[25] & 0x01;              // useDhcp
            data[29] = record.data[25] & 0x04;              // invertHeadDirection
            data[30] = 0;                                   // ahrsMode
            Mem::packFloat32(&data[31], 1);                 // orientationOffset.w
            Mem::packFloat32(&data[35], 0);                 // orientationOffset.x
            Mem::packFloat32(&data[39], 0);                 // orientationOffset.y
            Mem::packFloat32(&data[43], 0);                 // orientationOffset.z
            Mem::memcpy(&data[47], &record.data[26], 4);    // headingOffsetRad
            Mem::packFloat32(&data[51], 0);                 // turnsAbout.x
            Mem::packFloat32(&data[55], 0);                 // turnsAbout.y
            Mem::packFloat32(&data[59], 1);                 // turnsAbout.z
            data[63] = 0;                                   // turnsAboutEarthFrame
            Mem::packFloat32(&data[64], 0);                 // tvgPoints[0].x
            Mem::packFloat32(&data[68], 0);                 // tvgPoints[0].y
            Mem::packFloat32(&data[72], 0);                 // tvgPoints[1].x
            Mem::packFloat32(&data[76], 0);                 // tvgPoints[1].y
            Mem::packFloat32(&data[80], 0);                 // tvgPoints[2].x
            Mem::packFloat32(&data[84], 0);                 // tvgPoints[2].y
            Mem::packFloat32(&data[88], 0);                 // tvgPoints[3].x
            Mem::packFloat32(&data[92], 0);                 // tvgPoints[3].y
            Mem::packFloat32(&data[96], 0);                 // tvgPoints[4].x
            Mem::packFloat32(&data[100], 0);                // tvgPoints[4].y
            Mem::packFloat32(&data[104], 0);                // tvgPoints[5].x
            Mem::packFloat32(&data[108], 0);                // tvgPoints[5].y
            Mem::packFloat32(&data[112], 0);                // tvgPoints[6].x
            Mem::packFloat32(&data[116], 0);                // tvgPoints[6].y
            Mem::packFloat32(&data[120], 0);                // tvgPoints[7].x
            Mem::packFloat32(&data[124], 0);                // tvgPoints[7].y
            Mem::packFloat32(&data[128], 0);                // tvgPoints[8].x
            Mem::packFloat32(&data[132], 0);                // tvgPoints[8].y

            Mem::memcpy(&data[136], &record.data[42], 44);
        }
        break;
    }
    case DataType::Iss360SettingsV2:
    {
        if (record.data.size() == 179)
        {
            data.resize(180);
            data[0] = 0x8c;
            Mem::memcpy(&data[1], &record.data[0], 179);
        }
        break;
    }
    case DataType::Iss360PingV1:
    case DataType::Iss360PingV2:
    {
        uint16_t angle = Mem::get16Bit(&record.data[0]);
        uint32_t minRange = 0;
        uint32_t maxRange = 0;
        uint32_t dataCount = 0;
        uint_t idx = 0;

        if (dataTye == DataType::Iss360PingV2)
        {
            int32_t stepSize = static_cast<int32_t>(Mem::get32Bit(&record.data[2]));
            minRange = Mem::get32Bit(&record.data[6]);
            maxRange = Mem::get32Bit(&record.data[10]);
            minRange = (stepSize << 20) & 0xfff00000 | (minRange & 0x000fffff);
            dataCount = Mem::get32Bit(&record.data[14]);
            idx = 18;
        }
        else
        {
            minRange = Mem::get32Bit(&record.data[2]);
            maxRange = Mem::get32Bit(&record.data[6]);
            dataCount = Mem::get16Bit(&record.data[10]);
            idx = 12;
        }

        data.resize(11 + dataCount * 2);
        data[0] = 0x97;
        Mem::pack16Bit(&data[1], angle);
        Mem::pack32Bit(&data[3], minRange);
        Mem::pack32Bit(&data[7], maxRange);

        Mem::memcpy(&data[11], &record.data[idx], dataCount * 2);
        break;
    }

    default:
        break;
    }

    return data;
}
//--------------------------------------------------------------------------------------------------
