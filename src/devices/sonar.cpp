//------------------------------------------ Includes ----------------------------------------------

#include "sonar.h"
#include "maths/maths.h"
#include "platform/debug.h"
#include "platform/mem.h"
#include "utils/xmlSettings.h"
#include "utils/stringUtils.h"
#include "utils/utils.h"

using namespace IslSdk;

const uint_t Sonar::maxAngle;

//--------------------------------------------------------------------------------------------------
Sonar::Sonar(const Device::Info& info) : Device(info), m_macAddress{}
{
    m_requestedRates.ahrs = 100;
    m_requestedRates.gyro = 100;
    m_requestedRates.accel = 100;
    m_requestedRates.mag = 100;
    m_requestedRates.voltageAndTemp = 1000;

    ahrs.onData.setSubscribersChangedCallback(this, &Sonar::signalSubscribersChanged);
    gyro.onData.setSubscribersChangedCallback(this, &Sonar::signalSubscribersChanged);
    accel.onData.setSubscribersChangedCallback(this, &Sonar::signalSubscribersChanged);
    mag.onData.setSubscribersChangedCallback(this, &Sonar::signalSubscribersChanged);
}
//--------------------------------------------------------------------------------------------------
Sonar::~Sonar()
{
}
//--------------------------------------------------------------------------------------------------
void Sonar::setSensorRates(const SensorRates& rates)
{
    m_requestedRates = rates;

    if (m_connected)
    {
        SensorRates toSend = m_requestedRates;

        if (!ahrs.onData.hasSubscribers()) toSend.ahrs = 0;
        if (!gyro.onData.hasSubscribers()) toSend.gyro = 0;
        if (!accel.onData.hasSubscribers()) toSend.accel = 0;
        if (!mag.onData.hasSubscribers()) toSend.mag = 0;
        if (!onPwrAndTemp.hasSubscribers()) toSend.voltageAndTemp = 0;

        uint8_t data[21];
        uint8_t* buf = &data[0];

        *buf++ = static_cast<uint8_t>(Commands::SetSensorInterval);
        Mem::pack32Bit(&buf, toSend.ahrs);
        Mem::pack32Bit(&buf, toSend.gyro);
        Mem::pack32Bit(&buf, toSend.accel);
        Mem::pack32Bit(&buf, toSend.mag);
        Mem::pack32Bit(&buf, toSend.voltageAndTemp);

        enqueuePacket(&data[0], sizeof(data));
    }
}
//--------------------------------------------------------------------------------------------------
bool_t Sonar::setSystemSettings(const System& newSettings, bool_t save)
{
    uint8_t data[System::size + 2 + 72];
    uint8_t* buf = &data[0];
    std::vector<std::string> errMsgs;
    bool_t ok = newSettings.check(errMsgs);

    if (ok)
    {
        m_pendingSystemSettings = std::make_unique<System>(newSettings);

        *buf++ = static_cast<uint8_t>(Commands::SetSystemSettings);
        *buf++ = static_cast<uint8_t>(save);
        buf += newSettings.serialise(buf, sizeof(data) - 2);

        if (info.firmwareVersionBcd < 0x0310)
        {
            buf -= 22;
            for (uint_t i = 0; i < m_tvgPoints.size(); i++)
            {
                Mem::packFloat32(&buf, m_tvgPoints[i].x);
                Mem::packFloat32(&buf, m_tvgPoints[i].y);
            }
        }

        if (m_connection && m_connection->sysPort->type == SysPort::Type::Net)
        {
            if (newSettings.ipAddress != m_settings.system.ipAddress || newSettings.port != m_settings.system.port)
            {
                connectionSettingsUpdated(ConnectionMeta(newSettings.ipAddress, newSettings.port), false);
            }
        }
        else
        {
            if (newSettings.baudrate != m_settings.system.baudrate || newSettings.uartMode != m_settings.system.uartMode)
            {
                connectionSettingsUpdated(ConnectionMeta(newSettings.baudrate), newSettings.uartMode != UartMode::Rs232);
            }
        }
        enqueuePacket(&data[0], buf - &data[0]);
    }
    else
    {
        for (size_t i = 0; i < errMsgs.size(); i++)
        {
            onError(*this, "System setting " + errMsgs[i]);
        }
    }

    return ok;
}
//--------------------------------------------------------------------------------------------------
bool_t Sonar::setAcousticSettings(const Acoustic& newSettings, bool_t save)
{
    uint8_t data[Acoustic::size + 2];
    std::vector<std::string> errMsgs;
    bool_t ok = newSettings.check(errMsgs);

    if (ok)
    {
        m_pendingAcousticSettings = std::make_unique<Acoustic>(newSettings);

        data[0] = static_cast<uint8_t>(Commands::SetAcousticSettings);
        data[1] = static_cast<uint8_t>(save);
        newSettings.serialise(&data[2], sizeof(data) - 2);

        enqueuePacket(&data[0], sizeof(data));
    }
    else
    {
        for (size_t i = 0; i < errMsgs.size(); i++)
        {
            onError(*this, "Acoustic setting " + errMsgs[i]);
        }
    }
    return ok;
}
//--------------------------------------------------------------------------------------------------
bool_t Sonar::setSetupSettings(const Setup& newSettings, bool_t save)
{
    uint8_t data[Setup::size + 2];
    std::vector<std::string> errMsgs;
    bool_t ok = newSettings.check(errMsgs);

    if (ok)
    {
        m_pendingSetupSettings = std::make_unique<Setup>(newSettings);

        data[0] = static_cast<uint8_t>(Commands::SetSetupSettings);
        data[1] = static_cast<uint8_t>(save);
        newSettings.serialise(&data[2], sizeof(data) - 2);

        enqueuePacket(&data[0], sizeof(data));
    }
    else
    {
        for (size_t i = 0; i < errMsgs.size(); i++)
        {
            onError(*this, "Setup setting " + errMsgs[i]);
        }
    }
    return ok;
}
//--------------------------------------------------------------------------------------------------
void Sonar::homeHead()
{
    uint8_t data;

    data = static_cast<uint8_t>(Commands::HomeHead);
    enqueuePacket(&data, sizeof(data));
}
//--------------------------------------------------------------------------------------------------
void Sonar::startScanning()
{
    uint8_t data[2];
    data[0] = static_cast<uint8_t>(Commands::StopStart);
    data[1] = 1;

    enqueuePacket(&data[0], sizeof(data));
}
//--------------------------------------------------------------------------------------------------
void Sonar::stopScanning()
{
    uint8_t data[2];
    data[0] = static_cast<uint8_t>(Commands::StopStart);
    data[1] = 0;

    enqueuePacket(&data[0], sizeof(data));
}
//--------------------------------------------------------------------------------------------------
void Sonar::moveHead(int16_t angle, bool_t relative)
{
    if (info.firmwareVersionBcd < 0x0310) return;

    if (!relative)
    {
        angle = std::max<int16_t>(angle, 0);
        angle = std::min<int16_t>(angle, Sonar::maxAngle-1);
    }

    uint8_t data[4];
    data[0] = static_cast<uint8_t>(Commands::MoveMotor);
    Mem::pack16Bit(&data[1], angle);
    data[3] = static_cast<uint8_t>(relative);

    enqueuePacket(&data[0], sizeof(data));
}
//--------------------------------------------------------------------------------------------------
void Sonar::testPattern(bool_t enable)
{
    uint8_t data[2];

    data[0] = static_cast<uint8_t>(Commands::TestImage);
    data[1] = static_cast<uint8_t>(enable);
    if (info.firmwareVersionBcd < 0x0310) data[0] = 36;
    enqueuePacket(&data[0], sizeof(data));
}
//--------------------------------------------------------------------------------------------------
void Sonar::setTvg(const std::array<Point, 9>& points)
{
    m_tvgPoints = points;

    if (info.firmwareVersionBcd >= 0x0310)
    {
        uint8_t data[73];
        uint8_t* buf = &data[0];

        *buf++ = static_cast<uint8_t>(Commands::SetTvg);
        for (uint_t i = 0; i < points.size(); i++)
        {
            Mem::packFloat32(&buf, points[i].x);
            Mem::packFloat32(&buf, points[i].y);
        }
        enqueuePacket(&data[0], buf - &data[0]);
    }
    else
    {
        setSystemSettings(m_settings.system, true);
    }
}
//--------------------------------------------------------------------------------------------------
std::array<Point, 9> Sonar::getDefaultTvg()
{
    std::array<Point, 9> tvg;

    tvg[0].x = 0;
    tvg[0].y = 10.0;
    tvg[1].x = 25;
    tvg[1].y = 28.0;
    tvg[2].x = 50;
    tvg[2].y = 40.0;
    tvg[3].x = 75;
    tvg[3].y = 46.0;
    tvg[4].x = 100;
    tvg[4].y = 48.0;
    tvg[5].x = 125;
    tvg[5].y = 48.0;
    tvg[6].x = 150;
    tvg[6].y = 48.0;
    tvg[7].x = 175;
    tvg[7].y = 48.0;
    tvg[8].x = 200;
    tvg[8].y = 48.0;

    return tvg;
}

//--------------------------------------------------------------------------------------------------
bool_t Sonar::startLogging()
{
    Device::startLogging();
    return logSettings();
}
//--------------------------------------------------------------------------------------------------
bool_t Sonar::saveConfig(const std::string& fileName)
{
    bool_t ok = false;

    XmlFile file;
    XmlElementPtr rootXml = file.setRoot("SONAR");
    if (rootXml)
    {
        XmlSettings::saveDeviceInfo(info, rootXml);

        rootXml->addString("macAddress", StringUtils::macAddressToStr(m_macAddress));

        XmlElementPtr xml = rootXml->addElement("settings");
        m_settings.save(xml);

        xml = rootXml->addElement("cal");
        if (xml)
        {
            XmlElementPtr node = xml->addElement("gyro");
            XmlSettings::saveBias(gyro.bias, node);
            node = xml->addElement("accel");
            XmlSettings::saveBias(accel.bias, node);
            XmlSettings::saveTransform(accel.transform, node);
            node = xml->addElement("mag");
            XmlSettings::saveBias(mag.bias, node);
            XmlSettings::saveTransform(mag.transform, node);
        }

        xml = rootXml->addElement("tvg");
        for (uint_t i = 0; i < m_tvgPoints.size(); i++)
        {
            XmlElementPtr node = xml->addElement("point");
            node->addAttribute("id", StringUtils::uintToStr(i + 1));
            XmlElementPtr node1 = node->addReal("distance", m_tvgPoints[i].x, 6);
            node1->addAttribute("units", "m");
            node1 = node->addReal("amplitude", m_tvgPoints[i].y, 6);
            node1->addAttribute("units", "dB");
        }
        ok = file.save(fileName);
    }

    return ok;
}
//--------------------------------------------------------------------------------------------------
bool_t Sonar::loadConfig(const std::string& fileName, Device::Info* info, Settings* settings, AhrsCal* ahrsCal, std::array<Point, 9>* tvgPoints)
{
    bool_t ok = false;

    XmlFile file;
    if (file.open(fileName))
    {
        XmlElementPtr baseNode = file.root();
        if (baseNode && baseNode->name == "SONAR")
        {
            ok = true;
            if (info)
            {
                ok &= XmlSettings::loadDeviceInfo(*info, baseNode);
            }

            //StringUtils::toMacAddress(baseNode->getString("macAddress", "00:00:00:00:00:00"), macAddress);

            if (settings)
            {
                XmlElementPtr xml = baseNode->findElement("settings");
                ok &= settings->load(xml);
            }

            if (ahrsCal)
            {
                XmlElementPtr xml = baseNode->findElement("cal");
                if (xml)
                {
                    XmlElementPtr node = xml->findElement("gyro");
                    ok &= XmlSettings::loadBias(ahrsCal->gyroBias, node);

                    node = xml->findElement("accel");
                    ok &= XmlSettings::loadBias(ahrsCal->accelBias, node);
                    ok &= XmlSettings::loadTransform(ahrsCal->accelTransform, node);

                    node = xml->findElement("mag");
                    ok &= XmlSettings::loadBias(ahrsCal->magBias, node);
                    ok &= XmlSettings::loadTransform(ahrsCal->magTransform, node);
                }
            }

            if (tvgPoints)
            {
                XmlElementPtr xml = baseNode->findElement("tvg");
                if (xml)
                {
                    for (const XmlElementPtr& p : xml->elements)
                    {
                        if (p->name != "point") break;
                        bool_t error = false;
                        uint_t i = StringUtils::toUint(p->getAttribute("id"), error);
                        if (i && (i - 1) < (*tvgPoints).size())
                        {
                            real_t x = p->getReal("distance", -1000.0);
                            real_t y = p->getReal("amplitude", -1000.0);

                            if (x > -999.0 && y > -999.0)
                            {
                                (*tvgPoints)[i - 1].x = x;
                                (*tvgPoints)[i - 1].y = y;
                            }
                            else
                            {
                                break;
                            }
                        }
                    }
                }
            }
        }
    }

    return ok;
}
//--------------------------------------------------------------------------------------------------
void Sonar::connectionEvent(bool_t isConnected)
{
    if (isConnected)
    {
        if (bootloaderMode())
        {
            Device::connectionEvent(true);
        }
        else
        {
            setSensorRates(m_requestedRates);
            selectDataOutput(onPingData.hasSubscribers(), onEchoData.hasSubscribers());
            if (!m_connectionDataSynced)
            {
                getAhrsCal();
                getTvg();
                getSettings();
            }
            else
            {
                Device::connectionEvent(true);
            }
        }
    }
    else
    {
        Device::connectionEvent(false);
    }
}
//--------------------------------------------------------------------------------------------------
bool_t Sonar::newPacket(uint8_t command, const uint8_t* data, uint_t size)
{
    bool_t shouldLog = false;

    switch (static_cast<Commands>(command))
    {
    case Commands::GetSensorData:
    {
        shouldLog = true;
        uint_t sensorFlags = Mem::get32Bit(&data);

        if (sensorFlags & DataFlags::ahrs)
        {
            real_t w = Mem::getFloat32(&data);
            real_t x = Mem::getFloat32(&data);
            real_t y = Mem::getFloat32(&data);
            real_t z = Mem::getFloat32(&data);
            Quaternion q = Quaternion(w, x, y, z);
            real_t magHeadingRad = Mem::getFloat32(&data);
            real_t turnsCount = Mem::getFloat32(&data);
            ahrs.onData(ahrs, 0, q, magHeadingRad, turnsCount);
        }

        if (sensorFlags & DataFlags::gyro)
        {
            Vector3 gyroVec;
            gyroVec.x = Mem::getFloat32(&data);
            gyroVec.y = Mem::getFloat32(&data);
            gyroVec.z = Mem::getFloat32(&data);
            gyro.onData(gyro, gyroVec);
        }

        if (sensorFlags & DataFlags::accel)
        {
            Vector3 accelVec;
            accelVec.x = Mem::getFloat32(&data);
            accelVec.y = Mem::getFloat32(&data);
            accelVec.z = Mem::getFloat32(&data);
            accel.onData(accel, accelVec);
        }

        if (sensorFlags & DataFlags::mag)
        {
            Vector3 magVec;
            magVec.x = Mem::getFloat32(&data);
            magVec.y = Mem::getFloat32(&data);
            magVec.z = Mem::getFloat32(&data);
            mag.onData(mag, magVec);
        }

        if (sensorFlags & DataFlags::cpuTempPower)
        {
            CpuPowerTemp cpuPwrTemp;
            cpuPwrTemp.core1V0 = Mem::getFloat32(&data);
            cpuPwrTemp.aux1V8 = Mem::getFloat32(&data);
            cpuPwrTemp.ddr1V35 = Mem::getFloat32(&data);
            cpuPwrTemp.cpuTemperature = Mem::getFloat32(&data);
            cpuPwrTemp.auxTemperature = Mem::getFloat32(&data);
            onPwrAndTemp(*this, cpuPwrTemp);
        }
        break;
    }
    case Commands::SetSensorInterval:
    {
        break;
    }
    case Commands::GetSettings:
    {
        shouldLog = true;
        if (info.firmwareVersionBcd >= 0x0310)
        {
            Mem::memcpy(&m_macAddress[0], data, 6);
            data += 6;
            size -= 6;
            m_settings.deserialise(data, size);
        }
        else if (size >= 179)
        {
            Mem::memcpy(&m_macAddress[0], data, 6);
            data += 6;
            size -= 6;
            data += m_settings.system.deserialise(data, 57);
            for (uint_t i = 0; i < m_tvgPoints.size(); i++)
            {
                m_tvgPoints[i].x = Mem::getFloat32(&data);
                m_tvgPoints[i].y = Mem::getFloat32(&data);
            }
            data += m_settings.acoustic.deserialise(data, size - (57 + 72));
            data += m_settings.setup.deserialise(data, size - (57 + 72 + Acoustic::size));

            if (connection == nullptr)
            {
                onSettingsUpdated(*this, true, Settings::Type::System);
                onSettingsUpdated(*this, true, Settings::Type::Acoustic);
                onSettingsUpdated(*this, true, Settings::Type::Setup);
            }
        }
        if (!m_connectionDataSynced)
        {
            Device::connectionEvent(true);
        }
        break;
    }
    case Commands::SaveSettings:
    {
        break;
    }
    case Commands::SetSystemSettings:
    {
        shouldLog = true;
        if (*data == 0)
        {
            onError(*this, "Settings not applied or failed to save to device");
        }
        else
        {
            if (m_pendingSystemSettings)
            {
                m_settings.system = *m_pendingSystemSettings;
            }
            logSettings();
        }
        m_pendingSystemSettings.reset();
        onSettingsUpdated(*this, *data != 0, Settings::Type::System);
        break;
    }
    case Commands::SetAcousticSettings:
    {
        shouldLog = true;
        if (*data == 0)
        {
            onError(*this, "Settings not applied or failed to save to device");
        }
        else
        {
            if (m_pendingAcousticSettings)
            {
                m_settings.acoustic = *m_pendingAcousticSettings;
            }
            logSettings();
        }
        m_pendingAcousticSettings.reset();
        onSettingsUpdated(*this, *data != 0, Settings::Type::Acoustic);
        break;
    }
    case Commands::SetSetupSettings:
    {
        shouldLog = true;
        if (*data == 0)
        {
            onError(*this, "Settings not applied or failed to save to device");
        }
        else
        {
            if (m_pendingSetupSettings)
            {
                m_settings.setup = *m_pendingSetupSettings;
            }
            logSettings();
        }
        m_pendingSetupSettings.reset();
        onSettingsUpdated(*this, *data != 0, Settings::Type::Setup);
        break;
    }
    case Commands::GetAhrsCal:
    {
        Vector3 gyroVec, accelVec, magVec;
        Matrix3x3 accelMat, magMat;
        gyroVec.x = Mem::getFloat32(&data);
        gyroVec.y = Mem::getFloat32(&data);
        gyroVec.z = Mem::getFloat32(&data);
        accelVec.x = Mem::getFloat32(&data);
        accelVec.y = Mem::getFloat32(&data);
        accelVec.z = Mem::getFloat32(&data);
        magVec.x = Mem::getFloat32(&data);
        magVec.y = Mem::getFloat32(&data);
        magVec.z = Mem::getFloat32(&data);
        accelMat.m[0][0] = Mem::getFloat32(&data);
        accelMat.m[0][1] = Mem::getFloat32(&data);
        accelMat.m[0][2] = Mem::getFloat32(&data);
        accelMat.m[1][0] = Mem::getFloat32(&data);
        accelMat.m[1][1] = Mem::getFloat32(&data);
        accelMat.m[1][2] = Mem::getFloat32(&data);
        accelMat.m[2][0] = Mem::getFloat32(&data);
        accelMat.m[2][1] = Mem::getFloat32(&data);
        accelMat.m[2][2] = Mem::getFloat32(&data);
        magMat.m[0][0] = Mem::getFloat32(&data);
        magMat.m[0][1] = Mem::getFloat32(&data);
        magMat.m[0][2] = Mem::getFloat32(&data);
        magMat.m[1][0] = Mem::getFloat32(&data);
        magMat.m[1][1] = Mem::getFloat32(&data);
        magMat.m[1][2] = Mem::getFloat32(&data);
        magMat.m[2][0] = Mem::getFloat32(&data);
        magMat.m[2][1] = Mem::getFloat32(&data);
        magMat.m[2][2] = Mem::getFloat32(&data);
        gyro.updateCalValues(gyroVec);
        accel.updateCalValues(accelVec, accelMat);
        mag.updateCalValues(magVec, magMat);
        break;
    }
    case Commands::SetGyroCal:
    {
        if (*data == 0)
        {
            onError(*this, "Gyro cal failed to save to device");
        }
        break;
    }
    case Commands::SetAccelCal:
    {
        if (*data == 0)
        {
            onError(*this, "Accel cal failed to save to device");
        }
        break;
    }
    case Commands::SetMagCal:
    {
        if (*data == 0)
        {
            onError(*this, "Mag cal failed to save to device");
        }
        break;
    }
    case Commands::SetHeading:
    {
        break;
    }
    case Commands::ClearTurnsCount:
    {
        break;
    }
    case Commands::PingData:
    {
        shouldLog = true;
        size -= 10;
        uint16_t temp16 = Mem::get16Bit(&data);
        if (temp16 & 0xc000)
        {
            size *= 2;
        }

        Ping ping;
        ping.angle = temp16 & 0x3fff;
        ping.minRangeMm = Mem::get32Bit(&data);
        ping.maxRangeMm = Mem::get32Bit(&data);
        ping.stepSize = static_cast<int32_t>(ping.minRangeMm) >> 20;
        ping.minRangeMm &= 0x000fffff;

        uint_t dataCount = size / 2;
        ping.data.resize(dataCount);
        if (temp16 & 0xc000)
        {
            for (uint_t i = 0; i < dataCount; i++)
            {
                ping.data[i] = static_cast<uint16_t>(data[i]) << 8;
            }
        }
        else
        {
            Mem::memcpy(&ping.data[0], data, size);
        }

        if (ping.stepSize == 0)
        {
            ping.stepSize = settings.setup.stepSize;
        }

        onPingData(*this, ping);
        break;
    }
    case Commands::EchoData:
    {
        shouldLog = true;
        Echos echos;
        echos.angle = Mem::get16Bit(&data);
        echos.minRangeMm = Mem::get32Bit(&data);
        echos.maxRangeMm = Mem::get32Bit(&data);
        echos.timeUs = Mem::get48Bit(&data) + m_epochUs;
        uint_t count = *data++;
        echos.data.resize(count);

        for (uint_t i = 0; i < count; i++)
        {
            echos.data[i].correlation = Mem::getFloat32(&data);
            echos.data[i].signalEnergy = Mem::getFloat32(&data);
            echos.data[i].totalTof = Mem::getFloat32(&data);
        }
        onEchoData(*this, echos);
        break;
    }
    case Commands::HomeHead:
    {
        HeadHome home;
        home.state = static_cast<HeadHome::HeadHomeState>(*data++);
        
        onHeadHomed(*this, home);
        break;
    }
    case Commands::StopStart:
    {
        break;
    }
    case Commands::SetDataOptions:
    {
        break;
    }
    case Commands::MoveMotor:
    {
        break;
    }
    case Commands::TestImage:
    {
        break;
    }
    case Commands::GetTvg:
    {
        for (uint_t i = 0; i < m_tvgPoints.size(); i++)
        {
            m_tvgPoints[i].x = Mem::getFloat32(&data);
            m_tvgPoints[i].y = Mem::getFloat32(&data);
        }
        break;
    }
    case Commands::SetTvg:
    {
        if (*data == 0)
        {
            onError(*this, "TVG failed to save to device");
        }
        break;
    }
    
    default:
        break;
    }

    return shouldLog;
}
//--------------------------------------------------------------------------------------------------
void Sonar::signalSubscribersChanged(uint_t subscriberCount)
{
    if (subscriberCount <= 1)
    {
        setSensorRates(m_requestedRates);
    }
}
//--------------------------------------------------------------------------------------------------
void Sonar::sonarDataSignalSubscribersChanged(uint_t subscriberCount)
{
    if (subscriberCount <= 1)
    {
        selectDataOutput(onPingData.hasSubscribers(), onEchoData.hasSubscribers());
    }
}
//--------------------------------------------------------------------------------------------------
bool_t Sonar::logSettings()
{
    uint8_t data[Settings::size + 7];

    data[0] = static_cast<uint8_t>(Device::Commands::ReplyBit) | static_cast<uint8_t>(Commands::GetSettings);
    Mem::memcpy(&data[1], &m_macAddress[0], 6);
    m_settings.serialise(&data[7], sizeof(data) - 7);

    return log(&data[0], sizeof(data), static_cast<uint8_t>(LoggingDataType::packetData), false);
}
//--------------------------------------------------------------------------------------------------
void Sonar::getData(uint32_t flags)
{
    uint8_t data[5];

    data[0] = static_cast<uint8_t>(Commands::GetSensorData);
    Mem::pack32Bit(&data[1], flags);
    enqueuePacket(&data[0], sizeof(data));
}
//--------------------------------------------------------------------------------------------------
void Sonar::getSettings()
{
    uint8_t data = static_cast<uint8_t>(Commands::GetSettings);
    enqueuePacket(&data, sizeof(data));
}
//--------------------------------------------------------------------------------------------------
void Sonar::getAhrsCal()
{
    uint8_t data = static_cast<uint8_t>(Commands::GetAhrsCal);
    enqueuePacket(&data, sizeof(data));
}
//--------------------------------------------------------------------------------------------------
void Sonar::setGyroCal(uint_t sensorNum, const Vector3* bias)
{
    uint8_t data[13];
    uint8_t* buf = &data[0];

    *buf++ = static_cast<uint8_t>(Commands::SetGyroCal);

    if (bias)
    {
        Mem::packFloat32(&buf, bias->x);
        Mem::packFloat32(&buf, bias->y);
        Mem::packFloat32(&buf, bias->z);
    }

    enqueuePacket(&data[0], buf - &data[0]);
}
//--------------------------------------------------------------------------------------------------
void Sonar::setAccelCal(uint_t sensorNum, const Vector3& bias, const Matrix3x3& transform)
{
    uint8_t data[49];
    uint8_t* buf = &data[0];

    *buf++ = static_cast<uint8_t>(Commands::SetAccelCal);;
    Mem::packFloat32(&buf, bias.x);
    Mem::packFloat32(&buf, bias.y);
    Mem::packFloat32(&buf, bias.z);
    Mem::packFloat32(&buf, transform.m[0][0]);
    Mem::packFloat32(&buf, transform.m[0][1]);
    Mem::packFloat32(&buf, transform.m[0][2]);
    Mem::packFloat32(&buf, transform.m[1][0]);
    Mem::packFloat32(&buf, transform.m[1][1]);
    Mem::packFloat32(&buf, transform.m[1][2]);
    Mem::packFloat32(&buf, transform.m[2][0]);
    Mem::packFloat32(&buf, transform.m[2][1]);
    Mem::packFloat32(&buf, transform.m[2][2]);

    enqueuePacket(&data[0], sizeof(data));
}
//--------------------------------------------------------------------------------------------------
void Sonar::setMagCal(uint_t sensorNum, const Vector3& bias, const Matrix3x3& transform)
{
    uint8_t data[49];
    uint8_t* buf = &data[0];

    *buf++ = static_cast<uint8_t>(Commands::SetMagCal);
    Mem::packFloat32(&buf, bias.x);
    Mem::packFloat32(&buf, bias.y);
    Mem::packFloat32(&buf, bias.z);
    Mem::packFloat32(&buf, transform.m[0][0]);
    Mem::packFloat32(&buf, transform.m[0][1]);
    Mem::packFloat32(&buf, transform.m[0][2]);
    Mem::packFloat32(&buf, transform.m[1][0]);
    Mem::packFloat32(&buf, transform.m[1][1]);
    Mem::packFloat32(&buf, transform.m[1][2]);
    Mem::packFloat32(&buf, transform.m[2][0]);
    Mem::packFloat32(&buf, transform.m[2][1]);
    Mem::packFloat32(&buf, transform.m[2][2]);

    enqueuePacket(&data[0], sizeof(data));
}
//--------------------------------------------------------------------------------------------------
void Sonar::setHeading(const real_t* angleInRadians)
{
    uint8_t data[5];

    if (angleInRadians)
    {
        real_t angle = Math::fmod(*angleInRadians, Math::pi2);
        data[0] = static_cast<uint8_t>(Commands::SetHeading);
        Mem::packFloat32(&data[1], angle);
        enqueuePacket(&data[0], sizeof(data));
    }
    else
    {
        data[0] = static_cast<uint8_t>(Commands::SetHeading);
        enqueuePacket(&data[0], sizeof(data[0]));
    }
}
//--------------------------------------------------------------------------------------------------
void Sonar::clearTurnsCount()
{
    uint8_t data = static_cast<uint8_t>(Commands::ClearTurnsCount);
    enqueuePacket(&data, sizeof(data));
}
//--------------------------------------------------------------------------------------------------
void Sonar::selectDataOutput(bool_t ping, bool_t echo)
{
    if (m_connected && info.firmwareVersionBcd >= 0x0310)
    {
        uint8_t data[2];

        data[0] = static_cast<uint8_t>(Commands::SetDataOptions);
        data[1] = static_cast<uint8_t>(ping) | static_cast<uint8_t>(echo) << 1;
        enqueuePacket(&data[0], sizeof(data));
    }
}
//--------------------------------------------------------------------------------------------------
void Sonar::getTvg()
{
    if (info.firmwareVersionBcd >= 0x0310)
    {
        uint8_t data = static_cast<uint8_t>(Commands::GetTvg);
        enqueuePacket(&data, sizeof(data));
    }
}
//--------------------------------------------------------------------------------------------------
Sonar::Settings::Settings()
{
    defaults();
}
//--------------------------------------------------------------------------------------------------
void Sonar::Settings::defaults()
{
    system.defaults();
    setup.defaults();
    acoustic.defaults();
}
//--------------------------------------------------------------------------------------------------
uint_t Sonar::Settings::serialise(uint8_t* buf, uint_t sz) const
{
    if (sz >= size)
    {
        uint8_t* start = buf;
        buf += system.serialise(buf, System::size);
        buf += acoustic.serialise(buf, Acoustic::size);
        buf += setup.serialise(buf, Setup::size);

        return buf - start;
    }

    return 0;
}
//--------------------------------------------------------------------------------------------------
uint_t Sonar::Settings::deserialise(const uint8_t* data, uint_t sz)
{
    if (sz >= size)
    {
        const uint8_t* start = data;
        data += system.deserialise(data, System::size);
        data += acoustic.deserialise(data, Acoustic::size);
        data += setup.deserialise(data, Setup::size);

        return data - start;
    }

    return 0;
}
//--------------------------------------------------------------------------------------------------
bool_t Sonar::Settings::load(const XmlElementPtr& xml)
{
    bool_t ok = xml != nullptr;

    if (ok)
    {
        ok = system.load(xml);
        ok &= acoustic.load(xml);
        ok &= setup.load(xml);
    }
    return ok;
}
//--------------------------------------------------------------------------------------------------
void Sonar::Settings::save(XmlElementPtr& xml) const
{
    if (xml)
    {
        system.save(xml);
        acoustic.save(xml);
        setup.save(xml);
    }
}
//--------------------------------------------------------------------------------------------------
Sonar::System::System()
{
    defaults();
}
//--------------------------------------------------------------------------------------------------
void Sonar::System::defaults()
{
    uartMode = Device::UartMode::Rs232;
    baudrate = 115200;
    ipAddress = Utils::ipToUint(192, 168, 1, 200);
    netmask = Utils::ipToUint(255, 255, 255, 0);
    gateway = Utils::ipToUint(192, 168, 1, 1);
    port = 33005;
    phyPortMode = PhyPortMode::Auto;
    phyMdixMode = PhyMdixMode::Normal;
    useDhcp = false;
    invertHeadDirection = false;
    ahrsMode = 0;
    orientationOffset = { 1,0,0,0 };
    headingOffsetRad = 0;
    turnsAbout = { 0,0,1 };
    turnsAboutEarthFrame = false;

    echoMode = EchoMode::First;
    xcThreasholdLow = 0.25;
    xcThreasholdHigh = 0.3;
    energyThreashold = 0.0;
    useTiltCorrection = false;
    profilerDepthGating = false;
    useXcNorm = false;
    profilerMinRangeMm = 0;
    profilerMaxRangeMm = 10000;
}
//--------------------------------------------------------------------------------------------------
bool_t Sonar::System::check(std::vector<std::string>& errMsgs) const
{
    Utils::checkVar(uartMode, Device::UartMode::Rs232, Device::UartMode::Rs485Terminated, errMsgs, "uartMode out of range");
    Utils::checkVar<uint_t>(baudrate, 300, 115200, errMsgs, "baudrate out of range");
    Utils::checkVar<uint_t>(ahrsMode, 0, 1, errMsgs, "ahrsMode out of range");
    Utils::checkVar(phyPortMode, PhyPortMode::Auto, PhyPortMode::Base100TxFull, errMsgs, "phyPortMode out of range");
    Utils::checkVar(phyMdixMode, PhyMdixMode::Normal, PhyMdixMode::Auto, errMsgs, "phyMdixMode out of range");
    Utils::checkVar(orientationOffset.magnitude(), 0.99, 1.001, errMsgs, "orientationOffset quaternion is not normalised");
    Utils::checkVar(headingOffsetRad, -Math::pi2, Math::pi2, errMsgs, "headingOffsetRad out of range");
    Utils::checkVar(turnsAbout.magnitude(), 0.99, 1.001, errMsgs, "turnsAbout vector is not normalised");
    Utils::checkVar(echoMode, EchoMode::First, EchoMode::All, errMsgs, "echoMode out of range");
    Utils::checkVar(xcThreasholdLow, 0.0, 1.0, errMsgs, "xcThreasholdLow out of range");
    Utils::checkVar(xcThreasholdHigh, 0.0, 1.0, errMsgs, "xcThreasholdHigh out of range");
    Utils::checkVar(energyThreashold, 0.0, 1.0, errMsgs, "energyThreashold out of range");
    Utils::checkVar<int32_t>(profilerMaxRangeMm - profilerMinRangeMm, 1, 200000, errMsgs, "(profilerMaxRangeMm - profilerMinRangeMm) out of range");
    Utils::checkVar<uint32_t>(profilerMinRangeMm, 0, 200000, errMsgs, "profilerMinRangeMm out of range");
    Utils::checkVar<uint32_t>(profilerMaxRangeMm, 0, 200000, errMsgs, "profilerMaxRangeMm out of range");

    return errMsgs.empty();
}
//--------------------------------------------------------------------------------------------------
uint_t Sonar::System::serialise(uint8_t* buf, uint_t sz) const
{
    if (sz >= size)
    {
        uint8_t* start = buf;
        *buf++ = static_cast<uint8_t>(uartMode);
        Mem::pack32Bit(&buf, baudrate);
        Mem::pack32Bit(&buf, ipAddress);
        Mem::pack32Bit(&buf, netmask);
        Mem::pack32Bit(&buf, gateway);
        Mem::pack16Bit(&buf, port);
        *buf++ = static_cast<uint8_t>(phyPortMode);
        *buf++ = static_cast<uint8_t>(phyMdixMode);
        *buf++ = useDhcp;
        *buf++ = invertHeadDirection;
        *buf++ = ahrsMode;
        Mem::packFloat32(&buf, orientationOffset.w);
        Mem::packFloat32(&buf, orientationOffset.x);
        Mem::packFloat32(&buf, orientationOffset.y);
        Mem::packFloat32(&buf, orientationOffset.z);
        Mem::packFloat32(&buf, headingOffsetRad);
        Mem::packFloat32(&buf, turnsAbout.x);
        Mem::packFloat32(&buf, turnsAbout.y);
        Mem::packFloat32(&buf, turnsAbout.z);
        *buf++ = turnsAboutEarthFrame;

        *buf++ = static_cast<uint8_t>(echoMode);
        Mem::packFloat32(&buf, xcThreasholdLow);
        Mem::packFloat32(&buf, xcThreasholdHigh);
        Mem::packFloat32(&buf, energyThreashold);
        *buf++ = static_cast<uint8_t>(useTiltCorrection) | static_cast<uint8_t>(profilerDepthGating << 1) | static_cast<uint8_t>(useXcNorm << 2);
        Mem::pack32Bit(&buf, profilerMinRangeMm);
        Mem::pack32Bit(&buf, profilerMaxRangeMm);

        return buf - start;
    }

    return 0;
}
//--------------------------------------------------------------------------------------------------
uint_t Sonar::System::deserialise(const uint8_t* data, uint_t sz)
{
    if (sz >= baseSize)
    {
        const uint8_t* start = data;
        uartMode = static_cast<Device::UartMode>(*data++);
        baudrate = Mem::get32Bit(&data);
        ipAddress = Mem::get32Bit(&data);
        netmask = Mem::get32Bit(&data);
        gateway = Mem::get32Bit(&data);
        port = Mem::get16Bit(&data);
        phyPortMode = static_cast<Device::PhyPortMode>(*data++);
        phyMdixMode = static_cast<Device::PhyMdixMode>(*data++);
        useDhcp = *data++;
        invertHeadDirection = *data++;
        ahrsMode = *data++;
        orientationOffset.w = Mem::getFloat32(&data);
        orientationOffset.x = Mem::getFloat32(&data);
        orientationOffset.y = Mem::getFloat32(&data);
        orientationOffset.z = Mem::getFloat32(&data);
        headingOffsetRad = Mem::getFloat32(&data);
        turnsAbout.x = Mem::getFloat32(&data);
        turnsAbout.y = Mem::getFloat32(&data);
        turnsAbout.z = Mem::getFloat32(&data);
        turnsAboutEarthFrame = *data++;

        if ((sz - baseSize) >= profilerSize)
        {
            echoMode = static_cast<EchoMode>(*data++);
            xcThreasholdLow = Mem::getFloat32(&data);
            xcThreasholdHigh = Mem::getFloat32(&data);
            energyThreashold = Mem::getFloat32(&data);
            useTiltCorrection = (*data & 0x01) != 0;
            profilerDepthGating = (*data & 0x02) != 0;
            useXcNorm = (*data & 0x04) != 0;
            data++;
            profilerMinRangeMm = Mem::get32Bit(&data);
            profilerMaxRangeMm = Mem::get32Bit(&data);
        }
        else
        {
            echoMode = EchoMode::First;
            xcThreasholdLow = 0.25;
            xcThreasholdHigh = 0.3;
            energyThreashold = 0.0;
            useTiltCorrection = false;
            profilerDepthGating = false;
            useXcNorm = false;
            profilerMinRangeMm = 0;
            profilerMaxRangeMm = 10000;
        }

        return data - start;
    }

    return 0;
}
//--------------------------------------------------------------------------------------------------
bool_t Sonar::System::load(const XmlElementPtr& xml)
{
    bool_t ok = xml != nullptr;
    bool_t error = false;

    if (ok)
    {
        XmlElementPtr node = xml->findElement("system");
        if (node)
        {
            uartMode = StringUtils::toUartMode(node->getString("uartMode", "rs232"));
            baudrate = static_cast<uint32_t>(node->getUint("baudrate", 115200));
            ipAddress = StringUtils::toIp(node->getString("ipAddress", "192.168.1.200"), error);
            netmask = StringUtils::toIp(node->getString("netmask", "255.255.255.0"), error);
            gateway = StringUtils::toIp(node->getString("gateway", "192.168.1.1"), error);
            port = static_cast<uint16_t>(node->getUint("port", 33005));
            phyPortMode = StringUtils::toPhyPortMode(node->getString("phyPortMode", "phy auto"));
            phyMdixMode = StringUtils::toPhyMdixMode(node->getString("phyMdixMode", "mdix normal"));
            ahrsMode = static_cast<uint8_t>(node->getUint("ahrsMode", 0));
            headingOffsetRad = node->getReal("headingOffset", 0.0);

            XmlElementPtr node1 = node->findElement("orientationOffset");
            if (node1)
            {
                orientationOffset.w = node1->getReal("w", 0.0);
                orientationOffset.x = node1->getReal("x", 0.0);
                orientationOffset.y = node1->getReal("y", 0.0);
                orientationOffset.z = node1->getReal("z", 0.0);
            }
            node1 = node->findElement("turnsAbout");
            if (node1)
            {
                turnsAbout.x = node1->getReal("x", 0.0);
                turnsAbout.y = node1->getReal("y", 0.0);
                turnsAbout.z = node1->getReal("z", 0.0);
            }
            turnsAboutEarthFrame = node->getBool("turnsAboutEarthFrame", true);

            echoMode = static_cast<EchoMode>(node->getUint("echoMode", 0));
            xcThreasholdLow = node->getReal("xcThreasholdLow", 0.4);
            xcThreasholdHigh = node->getReal("xcThreasholdHigh", 0.5);
            energyThreashold = node->getReal("energyThreashold", 0.0);
            useTiltCorrection = node->getBool("useTiltCorrection", false);
            profilerDepthGating = node->getBool("profilerDepthGating", false);
            useXcNorm = node->getBool("useXcNorm", false);
            profilerMinRangeMm = static_cast<uint32_t>(node->getUint("profilerMinRangeMm", 0));
            profilerMaxRangeMm = static_cast<uint32_t>(node->getUint("profilerMaxRangeMm", 10000));
        }
    }
    return ok && !error;
}
//--------------------------------------------------------------------------------------------------
void Sonar::System::save(XmlElementPtr& xml) const
{
    if (xml)
    {
        XmlElementPtr node = xml->addElement("system");
        node->addString("uartMode", StringUtils::uartModeToStr(uartMode));
        node->addUint("baudrate", baudrate);
        node->addString("ipAddress", StringUtils::ipToStr(ipAddress));
        node->addString("netmask", StringUtils::ipToStr(netmask));
        node->addString("gateway", StringUtils::ipToStr(gateway));
        node->addUint("port", port);
        node->addString("phyPortMode", StringUtils::phyPortModeToStr(phyPortMode));
        node->addString("phyMdixMode", StringUtils::phyMdixModeToStr(phyMdixMode));
        node->addBool("useDhcp", useDhcp);
        node->addBool("invertHeadDirection", invertHeadDirection);
        node->addUint("ahrsMode", ahrsMode);
        node->addReal("headingOffset", headingOffsetRad, 3);

        XmlElementPtr node1 = node->addElement("orientationOffset");
        node1->addReal("w", orientationOffset.w, 6);
        node1->addReal("x", orientationOffset.x, 6);
        node1->addReal("y", orientationOffset.y, 6);
        node1->addReal("z", orientationOffset.z, 6);

        node1 = node->addElement("turnsAbout");
        node1->addReal("x", turnsAbout.x, 6);
        node1->addReal("y", turnsAbout.y, 6);
        node1->addReal("z", turnsAbout.z, 6);

        node->addBool("turnsAboutEarthFrame", turnsAboutEarthFrame);
        node->addUint("echoMode", static_cast<uint_t>(echoMode));
        node->addReal("xcThreasholdLow", xcThreasholdLow, 4);
        node->addReal("xcThreasholdHigh", xcThreasholdHigh, 4);
        node->addReal("energyThreashold", energyThreashold, 4);
        node->addBool("useTiltCorrection", useTiltCorrection);
        node->addBool("profilerDepthGating", profilerDepthGating);
        node->addBool("useXcNorm", useXcNorm);
        node->addUint("profilerMinRangeMm", profilerMinRangeMm);
        node->addUint("profilerMaxRangeMm", profilerMaxRangeMm);
    }
}
//--------------------------------------------------------------------------------------------------
Sonar::Acoustic::Acoustic()
{
    defaults();
}
//--------------------------------------------------------------------------------------------------
void Sonar::Acoustic::defaults()
{
    txStartFrequency = 650000;
    txEndFrequency = 750000;
    txPulseWidthUs = 200;
    txPulseAmplitude = 80;
    highSampleRate = true;
    pskMode = PskMode::Off;
}
//--------------------------------------------------------------------------------------------------
bool_t Sonar::Acoustic::check(std::vector<std::string>& errMsgs) const
{
    if (highSampleRate)
    {
        Utils::checkVar<uint32_t>(txStartFrequency, 200000, 2499000, errMsgs, "txStartFrequency out of range");
        Utils::checkVar<uint32_t>(txEndFrequency, 200000, 2499000, errMsgs, "txEndFrequency out of range");
        Utils::checkVar<uint16_t>(txPulseWidthUs, 0, 200, errMsgs, "txPulseWidthUs out of range");
    }
    else
    {
        Utils::checkVar<uint32_t>(txStartFrequency, 200000, 1249000, errMsgs, "txStartFrequency out of range");
        Utils::checkVar<uint32_t>(txEndFrequency, 200000, 1249000, errMsgs, "txEndFrequency out of range");
        Utils::checkVar<uint16_t>(txPulseWidthUs, 0, 400, errMsgs, "txPulseWidthUs out of range");
    }
    Utils::checkVar<uint8_t>(txPulseAmplitude, 0, 100, errMsgs, "txPulseAmplitude out of range");
    Utils::checkVar(pskMode, PskMode::Off, PskMode::Code4, errMsgs, "pskMode out of range");

    return errMsgs.empty();
}
//--------------------------------------------------------------------------------------------------
uint_t Sonar::Acoustic::serialise(uint8_t* buf, uint_t size) const
{
    if (size >= this->size)
    {
        uint8_t* start = buf;
        Mem::pack32Bit(&buf, txStartFrequency);
        Mem::pack32Bit(&buf, txEndFrequency);
        Mem::pack16Bit(&buf, txPulseWidthUs);
        *buf++ = txPulseAmplitude;
        *buf++ = static_cast<uint8_t>(highSampleRate) | ((static_cast<uint8_t>(pskMode) << 1) & 0x07);
        return buf - start;
    }

    return 0;
}
//--------------------------------------------------------------------------------------------------
uint_t Sonar::Acoustic::deserialise(const uint8_t* data, uint_t size)
{
    if (size >= this->size)
    {
        const uint8_t* start = data;
        txStartFrequency = Mem::get32Bit(&data);
        txEndFrequency = Mem::get32Bit(&data);
        txPulseWidthUs = Mem::get16Bit(&data);
        txPulseAmplitude = *data++;
        highSampleRate = (*data & 0x01) != 0;
        pskMode = static_cast<Sonar::Acoustic::PskMode>((*data >> 1) & 0x07);
        data++;

        return data - start;
    }

    return 0;
}
//--------------------------------------------------------------------------------------------------
bool_t Sonar::Acoustic::load(const XmlElementPtr& xml)
{
    bool_t ok = xml != nullptr;

    if (ok)
    {
        XmlElementPtr node = xml->findElement("acoustic");
        if (node)
        {
            txStartFrequency = static_cast<uint32_t>(node->getUint("txStartFrequency", 650000));
            txEndFrequency = static_cast<uint32_t>(node->getUint("txEndFrequency", 750000));
            txPulseWidthUs = static_cast<uint16_t>(node->getUint("txPulseWidthUs", 200));
            txPulseAmplitude = static_cast<uint8_t>(node->getUint("txPulseAmplitude", 80));
            highSampleRate = node->getBool("highSampleRate", false);
            pskMode = static_cast<Sonar::Acoustic::PskMode>(node->getUint("pskMode", 0));
        }
    }
    return ok;
}
//--------------------------------------------------------------------------------------------------
void Sonar::Acoustic::save(XmlElementPtr& xml) const
{
    if (xml)
    {
        XmlElementPtr node = xml->addElement("acoustic");
        node->addUint("txStartFrequency", txStartFrequency);
        node->addUint("txEndFrequency", txEndFrequency);
        node->addUint("txPulseWidthUs", txPulseWidthUs);
        node->addUint("txPulseAmplitude", txPulseAmplitude);
        node->addBool("highSampleRate", highSampleRate);
        node->addUint("pskMode", static_cast<uint_t>(pskMode));
    }
}
//--------------------------------------------------------------------------------------------------
Sonar::Setup::Setup()
{
    defaults();
}
//--------------------------------------------------------------------------------------------------
void Sonar::Setup::defaults()
{
    digitalGain = 1.0;
    speedOfSound = 1482.0;
    stepSize = 32;
    sectorStart = 0;
    sectorSize = 0;
    flybackMode = false;
    minRangeMm = 0;
    maxRangeMm = 10000;
    imageDataPoint = 200;
    data8Bit = false;
}
//--------------------------------------------------------------------------------------------------
bool_t Sonar::Setup::check(std::vector<std::string>& errMsgs) const
{
    Utils::checkVar(speedOfSound, 1000.0, 2500.0, errMsgs, "speedOfSound out of range");
    Utils::checkVar(digitalGain, 0.1, 1000.0, errMsgs, "digitalGain out of range");
    Utils::checkVar<int32_t>(stepSize, -6399, 6399, errMsgs, "stepSize out of range");
    Utils::checkVar<uint32_t>(sectorStart, 0, 12799, errMsgs, "sectorStart out of range");
    Utils::checkVar<uint32_t>(sectorSize, 0, 12800, errMsgs, "sectorSize out of range");
    Utils::checkVar<int32_t>(maxRangeMm - minRangeMm, 1, 200000, errMsgs, "(maxRangeMm - minRangeMm) out of range");
    Utils::checkVar<uint32_t>(minRangeMm, 0, 200000, errMsgs, "minRangeMm out of range");
    Utils::checkVar<uint32_t>(maxRangeMm, 0, 200000, errMsgs, "maxRangeMm out of range");
    Utils::checkVar<uint16_t>(imageDataPoint, 20, 4096, errMsgs, "imageDataPoint out of range");

    return errMsgs.empty();
}
//--------------------------------------------------------------------------------------------------
uint_t Sonar::Setup::serialise(uint8_t* buf, uint_t size) const
{
    if (size >= this->size)
    {
        uint8_t* start = buf;
        Mem::packFloat32(&buf, digitalGain);
        Mem::packFloat32(&buf, speedOfSound);
        Mem::pack32Bit(&buf, stepSize);
        Mem::pack32Bit(&buf, sectorStart);
        Mem::pack32Bit(&buf, sectorSize);
        *buf++ = flybackMode;
        Mem::pack32Bit(&buf, minRangeMm);
        Mem::pack32Bit(&buf, maxRangeMm);
        Mem::pack16Bit(&buf, imageDataPoint);
        *buf++ = data8Bit;

        return buf - start;
    }

    return 0;
}
//--------------------------------------------------------------------------------------------------
uint_t Sonar::Setup::deserialise(const uint8_t* data, uint_t size)
{
    if (size >= this->size)
    {
        const uint8_t* start = data;
        digitalGain = Mem::getFloat32(&data);
        speedOfSound = Mem::getFloat32(&data);
        stepSize = static_cast<int32_t>(Mem::get32Bit(&data));
        sectorStart = Mem::get32Bit(&data);
        sectorSize = Mem::get32Bit(&data);
        flybackMode = *data++;
        minRangeMm = Mem::get32Bit(&data);
        maxRangeMm = Mem::get32Bit(&data);
        imageDataPoint = Mem::get16Bit(&data);
        data8Bit = *data != 0;

        return data - start;
    }

    return 0;
}
//--------------------------------------------------------------------------------------------------
bool_t Sonar::Setup::load(const XmlElementPtr& xml)
{
    bool_t ok = xml != nullptr;

    if (ok)
    {
        XmlElementPtr node = xml->findElement("setup");
        if (node)
        {
            digitalGain = node->getReal("digitalGain", 1.0);
            speedOfSound = node->getReal("speedOfSound", 1482.0);
            stepSize = static_cast<int32_t>(node->getInt("stepSize", 32));
            sectorStart = static_cast<uint32_t>(node->getUint("sectorStart", 0));
            sectorSize = static_cast<uint32_t>(node->getUint("sectorSize", 0));
            flybackMode = node->getBool("flybackMode", false);
            minRangeMm = static_cast<uint32_t>(node->getUint("minRangeMm", 0));
            maxRangeMm = static_cast<uint32_t>(node->getUint("maxRangeMm", 100000));
            imageDataPoint = static_cast<uint16_t>(node->getUint("imageDataPoints", 200));
            data8Bit = node->getBool("data8Bit", false);
        }
    }
    return ok;
}
//--------------------------------------------------------------------------------------------------
void Sonar::Setup::save(XmlElementPtr& xml) const
{
    if (xml)
    {
        XmlElementPtr node = xml->addElement("setup");
        node->addReal("digitalGain", digitalGain, 2);
        node->addReal("speedOfSound", speedOfSound, 2);
        node->addInt("stepSize", stepSize);
        node->addUint("sectorStart", sectorStart);
        node->addUint("sectorSize", sectorSize);
        node->addBool("flybackMode", flybackMode);
        node->addUint("minRangeMm", minRangeMm);
        node->addUint("maxRangeMm", maxRangeMm);
        node->addUint("imageDataPoints", imageDataPoint);
        node->addBool("data8Bit", data8Bit);
    }
}
//--------------------------------------------------------------------------------------------------
