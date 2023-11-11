//------------------------------------------ Includes ----------------------------------------------

#include "isa500.h"
#include "maths/maths.h"
#include "platform/debug.h"
#include "platform/mem.h"
#include "files/xmlFile.h"
#include "utils/stringUtils.h"
#include "utils/xmlSettings.h"
#include "utils/utils.h"

using namespace IslSdk;

//--------------------------------------------------------------------------------------------------
Isa500::Isa500(const Device::Info& info) : Device(info)
{
    m_requestedRates.ping = 1000;
    m_requestedRates.ahrs = 0;
    m_requestedRates.gyro = 0;
    m_requestedRates.accel = 0;
    m_requestedRates.mag = 0;
    m_requestedRates.temperature = 1000;
    m_requestedRates.voltage = 1000;

    m_echogramDataPointCount = 0;

    ahrs.onData.setSubscribersChangedCallback(this, &Isa500::signalSubscribersChanged);
    gyro.onData.setSubscribersChangedCallback(this, &Isa500::signalSubscribersChanged);
    accel.onData.setSubscribersChangedCallback(this, &Isa500::signalSubscribersChanged);
    mag.onData.setSubscribersChangedCallback(this, &Isa500::signalSubscribersChanged);
}
//--------------------------------------------------------------------------------------------------
Isa500::~Isa500()
{
}
//--------------------------------------------------------------------------------------------------
void Isa500::setSensorRates(const SensorRates& rates)
{
    m_requestedRates = rates;

    if (m_connected)
    {
        SensorRates toSend = m_requestedRates;

        if (!onEcho.hasSubscribers() && !onEchogramData.hasSubscribers()) toSend.ping = 0;
        if (!ahrs.onData.hasSubscribers()) toSend.ahrs = 0;
        if (!gyro.onData.hasSubscribers()) toSend.gyro = 0;
        if (!accel.onData.hasSubscribers()) toSend.accel = 0;
        if (!mag.onData.hasSubscribers()) toSend.mag = 0;
        if (!onTemperature.hasSubscribers()) toSend.temperature = 0;
        if (!onVoltage.hasSubscribers()) toSend.voltage = 0;

        uint8_t data[29];
        uint8_t* buf = &data[0];

        *buf++ = static_cast<uint8_t>(Commands::SetSensorInterval);
        Mem::pack32Bit(&buf, toSend.ping);
        Mem::pack32Bit(&buf, toSend.ahrs);
        Mem::pack32Bit(&buf, toSend.gyro);
        Mem::pack32Bit(&buf, toSend.accel);
        Mem::pack32Bit(&buf, toSend.mag);
        Mem::pack32Bit(&buf, toSend.temperature);
        Mem::pack32Bit(&buf, toSend.voltage);

        enqueuePacket(&data[0], sizeof(data));
    }
}
//--------------------------------------------------------------------------------------------------
bool_t Isa500::setSettings(const Settings& newSettings, bool_t save)
{
    uint8_t data[Settings::size + 2];
    std::vector<std::string> errMsgs;
    bool_t ok = newSettings.check(errMsgs);

    if (ok)
    {
        m_pendingSettings = std::make_unique<Settings>(newSettings);

        data[0] = static_cast<uint8_t>(Commands::SetSettings);
        data[1] = static_cast<uint8_t>(save);
        newSettings.serialise(&data[2], sizeof(data) - 2);

        if (newSettings.baudrate != m_settings.baudrate || newSettings.uartMode != m_settings.uartMode)
        {
            connectionSettingsUpdated(ConnectionMeta(newSettings.baudrate), newSettings.uartMode != UartMode::Rs232);
        }
        enqueuePacket(&data[0], sizeof(data));
    }
    else
    {
        for (size_t i = 0; i < errMsgs.size(); i++)
        {
            onError(*this, "Setting " + errMsgs[i]);
        }
    }
    return ok;
}
//--------------------------------------------------------------------------------------------------
void Isa500::pingNow()
{
    getData(DataFlags::ping);
}
//--------------------------------------------------------------------------------------------------
void Isa500::setEchoGram(uint_t dataPointCount)
{
    m_echogramDataPointCount = Math::min<uint_t>(dataPointCount, 2000);

    if (m_connected)
    {
        uint8_t data[5];
        data[0] = static_cast<uint8_t>(Commands::EchogramData);
        Mem::pack32Bit(&data[1], onEchogramData.hasSubscribers() ? static_cast<uint32_t>(m_echogramDataPointCount) : 0);
        enqueuePacket(&data[0], sizeof(data));
    }
}
//--------------------------------------------------------------------------------------------------
bool_t Isa500::setPingScript(const std::string& name, const std::string& code)
{
    if (setScript(0, name, code))
    {
        m_onPing.name = name;
        m_onPing.code = code;
        m_onPing.state = DataState::Valid;
        return true;
    }
    return false;
}
//--------------------------------------------------------------------------------------------------
bool_t Isa500::setAhrsScript(const std::string& name, const std::string& code)
{
    if (setScript(1, name, code))
    {
        m_onAhrs.name = name;
        m_onAhrs.code = code;
        m_onAhrs.state = DataState::Valid;
        return true;
    }
    return false;
}
//--------------------------------------------------------------------------------------------------
bool_t Isa500::getScripts()
{
    if (m_scriptVars.state == DataState::Invalid)
    {
        m_scriptVars.state = DataState::Pending;
        getScriptVars();
    }

    if (m_onPing.state == DataState::Invalid)
    {
        m_onPing.state = DataState::Pending;
        getScript(0);
    }

    if (m_onAhrs.state == DataState::Invalid)
    {
        m_onAhrs.state = DataState::Pending;
        getScript(1);
    }

    return m_scriptVars.state == DataState::Valid && m_onPing.state == DataState::Valid && m_onAhrs.state == DataState::Valid;
}

//--------------------------------------------------------------------------------------------------
bool_t Isa500::saveConfig(const std::string& fileName)
{
    bool_t ok = true;

    getScripts();

    if (m_onPing.state == DataState::Valid && m_onAhrs.state == DataState::Valid)
    {
        XmlFile file;

        XmlElementPtr rootXml = file.setRoot("ISA500");
        if (rootXml)
        {
            XmlSettings::saveDeviceInfo(info, rootXml);

            XmlElementPtr xml = rootXml->addElement("settings");
            m_settings.save(xml);

            xml = rootXml->addElement("script0");
            XmlSettings::saveScript(m_onPing, xml);

            xml = rootXml->addElement("script1");
            XmlSettings::saveScript(m_onAhrs, xml);

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
            ok = file.save(fileName);
        }
    }
    else
    {
        m_saveConfigPath = fileName;
    }

    return ok;
}
//--------------------------------------------------------------------------------------------------
bool_t Isa500::loadConfig(const std::string& fileName, Device::Info* info, Settings* settings, DeviceScript* script0, DeviceScript* script1, AhrsCal* ahrsCal)
{
    bool_t ok = false;

    XmlFile file;
    if (file.open(fileName))
    {
        XmlElementPtr baseNode = file.findElementOnRoot("ISA500");
        if (baseNode)
        {
            ok = true;
            if (info)
            {
                ok &= XmlSettings::loadDeviceInfo(*info, baseNode);
            }

            if (settings)
            {
                XmlElementPtr xml = baseNode->findElement("settings");
                ok &= settings->load(xml);
            }

            if (script0)
            {
                XmlElementPtr xml = baseNode->findElement("script0");
                ok &= XmlSettings::loadScript(*script0, xml);
            }

            if (script1)
            {
                XmlElementPtr xml = baseNode->findElement("script1");
                ok &= XmlSettings::loadScript(*script1, xml);
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
        }
    }

    return ok;
}
//--------------------------------------------------------------------------------------------------
void Isa500::startLogging()
{
    Device::startLogging();
    logSettings();
}
//--------------------------------------------------------------------------------------------------
void Isa500::connectionEvent(bool_t isConnected)
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
            setEchoGram(m_echogramDataPointCount);
            if (!m_connectionDataSynced)
            {
                getStringNames(0);
                getStringNames(1);
                getAhrsCal();
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
bool_t Isa500::newPacket(uint8_t command, const uint8_t* data, uint_t size)
{
    bool_t shouldLog = false;

    switch (static_cast<Commands>(command))
    {
    case Commands::GetSensorData:
    {
        shouldLog = true;
        uint_t sensorFlags = Mem::get32Bit(&data);

        if (sensorFlags & DataFlags::ping)
        {
            uint64_t timeUs = Mem::get48Bit(&data) + m_epochUs;
            uint_t count = *data++;
            uint_t idx = *data++;
            uint_t totalEchoCount = Mem::get16Bit(&data);
            std::vector<Echo> echo(count);

            for (uint_t i = 0; i < count; i++)
            {
                echo[i].totalTof = Mem::getFloat32(&data);
                echo[i].correlation = Mem::getFloat32(&data);
                echo[i].signalEnergy = Mem::getFloat32(&data);
            }
            onEcho(*this, timeUs, idx, totalEchoCount, echo);
        }

        if (sensorFlags & DataFlags::ahrs)
        {
            uint64_t timeUs = Mem::get48Bit(&data) + m_epochUs;
            real_t w = Mem::getFloat32(&data);
            real_t x = Mem::getFloat32(&data);
            real_t y = Mem::getFloat32(&data);
            real_t z = Mem::getFloat32(&data);
            Quaternion q = Quaternion(w, x, y, z);
            real_t magHeadingRad = Mem::getFloat32(&data);
            real_t turnsCount = Mem::getFloat32(&data);
            ahrs.onData(ahrs, timeUs, q, magHeadingRad, turnsCount);
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

        if (sensorFlags & DataFlags::temperature)
        {
            onTemperature(*this, Mem::getFloat32(&data));
        }

        if (sensorFlags & DataFlags::voltage)
        {
            onVoltage(*this, Mem::getFloat32(&data));
        }

        if (sensorFlags & DataFlags::trigger)
        {
            onTrigger(*this, *data != 0);
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
        if (m_settings.deserialise(data, size))
        {
            if (!m_connectionDataSynced)
            {
                Device::connectionEvent(true);
            }
        }
        break;
    }
    case Commands::SetSettings:
    {
        shouldLog = true;
        if (*data == 0)
        {
            onError(*this, "Settings not applied or failed to save to device");
        }
        else
        {
            if (m_pendingSettings)
            {
                m_settings = *m_pendingSettings;
            }
            logSettings();
        }
        m_pendingSettings.reset();
        onSettingsUpdated(*this, *data != 0);
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
    case Commands::GetStringNames:
    {
        uint_t idx = *data++;
        uint_t count = *data++;
        size -= 2;

        if (idx == 0)
        {
            m_hardCodedPingOutputStrings.clear();
            for (uint_t i = 0; i < count; i++)
            {
                std::string str = StringUtils::toStr(data, size);
                m_hardCodedPingOutputStrings.emplace_back(str);
                data += str.size() + 1;
                size -= str.size() + 1;
            }
        }
        else if (idx == 1)
        {
            m_hardCodedAhrsOutputStrings.clear();
            for (uint_t i = 0; i < count; i++)
            {
                std::string str = StringUtils::toStr(data, size);
                m_hardCodedAhrsOutputStrings.emplace_back(str);
                data += str.size() + 1;
                size -= str.size() + 1;
            }
        }
        break;
    }
    case Commands::GetScriptVars:
    {
        uint_t count = *data++;
        size--;
        m_scriptVars.vars.clear();

        for (uint_t i = 0; i < count; i++)
        {
            ScriptVars::Var::Type varType = static_cast<ScriptVars::Var::Type>(*data++);
            size--;
            std::string name = StringUtils::toStr(data, size);
            data += name.size() + 1;
            size -= name.size() + 1;
            std::string description = StringUtils::toStr(data, size);
            data += description.size() + 1;
            size -= description.size() + 1;
            m_scriptVars.vars.emplace_back(name, description, varType);
        }
        m_scriptVars.state = DataState::Valid;

        if (m_scriptVars.state == DataState::Valid && m_onPing.state == DataState::Valid && m_onAhrs.state == DataState::Valid)
        {
            onScriptDataReceived(*this);
        }
        break;
    }
    case Commands::GetScript:
    {
        uint_t idx = *data++;
        size--;

        if (idx == 0)
        {
            m_onPing.name = StringUtils::toStr(data, size);
            data += m_onPing.name.size() + 1;
            size -= m_onPing.name.size() + 1;
            m_onPing.code = StringUtils::toStr(data, size);
            m_onPing.state = DataState::Valid;
        }
        else if (idx == 1)
        {
            m_onAhrs.name = StringUtils::toStr(data, size);
            data += m_onAhrs.name.size() + 1;
            size -= m_onAhrs.name.size() + 1;
            m_onAhrs.code = StringUtils::toStr(data, size);
            m_onAhrs.state = DataState::Valid;
        }

        if (m_scriptVars.state == DataState::Valid && m_onPing.state == DataState::Valid && m_onAhrs.state == DataState::Valid)
        {
            if (!m_saveConfigPath.empty())
            {
                saveConfig(m_saveConfigPath);
                m_saveConfigPath.clear();
            }
            onScriptDataReceived(*this);
        }
        break;
    }
    case Commands::SetScript:
    {
        if (data[1] == 0)
        {
            onError(*this, "Script " + StringUtils::uintToStr(data[0]) + " failed to save to device");
        }
        break;
    }
    
    case Commands::EchogramData:
    {
        shouldLog = true;
        std::vector<uint8_t> wave(data, &data[size]);
        onEchogramData(*this, wave);
        break;
    }
    default:
        break;
    }

    return shouldLog;
}
//--------------------------------------------------------------------------------------------------
void Isa500::signalSubscribersChanged(uint_t subscriberCount)
{
    if (subscriberCount <= 1)
    {
        setSensorRates(m_requestedRates);
    }
}
//--------------------------------------------------------------------------------------------------
void Isa500::echogramSignalSubscribersChanged(uint_t subscriberCount)
{
    if (subscriberCount <= 1)
    {
        setEchoGram(m_echogramDataPointCount);
        setSensorRates(m_requestedRates);
    }
}
//--------------------------------------------------------------------------------------------------
void Isa500::logSettings()
{
    if (islogging())
    {
        uint8_t data[Settings::size + 1];

        data[0] = static_cast<uint8_t>(Device::Commands::ReplyBit) | static_cast<uint8_t>(Commands::GetSettings);
        m_settings.serialise(&data[1], sizeof(data) - 1);

        log(&data[0], sizeof(data), static_cast<uint8_t>(LoggingDataType::packetData), false);
    }
}
//--------------------------------------------------------------------------------------------------
void Isa500::getData(uint32_t flags)
{
    uint8_t data[5];

    data[0] = static_cast<uint8_t>(Commands::GetSensorData);
    Mem::pack32Bit(&data[1], flags);
    enqueuePacket(&data[0], sizeof(data));
}
//--------------------------------------------------------------------------------------------------
void Isa500::getSettings()
{
    uint8_t data = static_cast<uint8_t>(Commands::GetSettings);
    enqueuePacket(&data, sizeof(data));
}
//--------------------------------------------------------------------------------------------------
void Isa500::getAhrsCal()
{
    uint8_t data = static_cast<uint8_t>(Commands::GetAhrsCal);
    enqueuePacket(&data, sizeof(data));
}
//--------------------------------------------------------------------------------------------------
void Isa500::setGyroCal(uint_t sensorNum, const Vector3* bias)
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
void Isa500::setAccelCal(uint_t sensorNum, const Vector3& bias, const Matrix3x3& transform)
{
    uint8_t data[49];
    uint8_t* buf = &data[0];

    *buf++ = Commands::SetAccelCal;
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
void Isa500::setMagCal(uint_t sensorNum, const Vector3& bias, const Matrix3x3& transform)
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
void Isa500::setHeading(const real_t* angleInRadians)
{
    uint8_t data[5];

    data[0] = static_cast<uint8_t>(Commands::SetHeading);

    if (angleInRadians)
    {
        real_t angle = Math::fmod(*angleInRadians, Math::pi2);
        Mem::packFloat32(&data[1], angle);
        enqueuePacket(&data[0], sizeof(data));
    }
    else
    {
        enqueuePacket(&data[0], sizeof(data[0]));
    }
}
//--------------------------------------------------------------------------------------------------
void Isa500::clearTurnsCount()
{
    uint8_t data = static_cast<uint8_t>(Commands::ClearTurnsCount);
    enqueuePacket(&data, sizeof(data));
}
//--------------------------------------------------------------------------------------------------
void Isa500::getStringNames(uint_t listId)
{
    uint8_t data[2];

    data[0] = static_cast<uint8_t>(Commands::GetStringNames);
    data[1] = static_cast<uint8_t>(listId);
    enqueuePacket(&data[0], sizeof(data));
}
//--------------------------------------------------------------------------------------------------
void Isa500::getScriptVars()
{
    uint8_t data = static_cast<uint8_t>(Commands::GetScriptVars);
    enqueuePacket(&data, sizeof(data));
}
//--------------------------------------------------------------------------------------------------
void Isa500::getScript(uint_t number)
{
    uint8_t data[2];

    data[0] = static_cast<uint8_t>(Commands::GetScript);
    data[1] = static_cast<uint8_t>(number);
    enqueuePacket(&data[0], sizeof(data));
}
//--------------------------------------------------------------------------------------------------
bool_t Isa500::setScript(uint_t number, const std::string& name, const std::string& code)
{
    uint8_t data[1024];
    uint8_t* buf = &data[0];

    if ((4 + name.size() + code.size()) <= sizeof(data))
    {
        *buf++ = static_cast<uint8_t>(Commands::SetScript);
        *buf++ = static_cast<uint8_t>(number);
        Mem::memcpy(buf, name.c_str(), name.size());
        buf += name.size();
        *buf++ = 0;

        Mem::memcpy(buf, code.c_str(), code.size());
        buf += code.size();
        *buf++ = 0;

        enqueuePacket(&data[0], (buf - &data[0]));
        return true;
    }
    return false;
}
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
Isa500::Settings::Settings()
{
    defaults();
}
//--------------------------------------------------------------------------------------------------
void Isa500::Settings::defaults()
{
    uartMode = Device::UartMode::Rs232;
    baudrate = 9600;
    parity = Device::Parity::None;
    dataBits = 8;
    stopBits = Device::StopBits::One;
    ahrsMode = 0;
    orientationOffset = { 1,0,0,0 };
    headingOffsetRad = 0;
    turnsAbout = { 0,0,1 };
    turnsAboutEarthFrame = false;
    clrTurn = { false, 3, {reinterpret_cast<const uint8_t*>("#c\r") } };
    setHeading2Mag = { false, 3, {reinterpret_cast<const uint8_t*>("#m\r")} };
    multiEchoLimit = 10;
    frequency = 500000;
    txPulseWidthUs = 200;
    txPulseAmplitude = 80;
    echoAnalyseMode = EchoAnalyseMode::Strongest;
    xcThreasholdLow = 0.4;
    xcThreasholdHigh = 0.5;
    energyThreashold = 0;
    speedOfSound = 1482.0;
    minRange = 0.7;
    maxRange = 120;
    distanceOffset = 0.0;
    useTiltCorrection = false;
    useMaxValueOnNoReturn = false;
    anaMode = AnalogueOutMode::None;
    aOutMinRange = 0;
    aOutMaxRange = 120;
    aOutMinVal = 0;
    aOutMaxVal = 5;
    strTrigger[0] = { 0, false, 500, false, false, { false, 3, reinterpret_cast<const uint8_t*>("#p\r") } };
    strTrigger[1] = { 0, false, 500, false, false, { false, 3, reinterpret_cast<const uint8_t*>("#o\r") } };
}
//--------------------------------------------------------------------------------------------------
bool_t Isa500::Settings::check(std::vector<std::string>& errMsgs) const
{
    Utils::checkVar(uartMode, Device::UartMode::Rs232, Device::UartMode::Rs485Terminated, errMsgs, "uartMode out of range");
    Utils::checkVar<uint_t>(baudrate, 300, 115200, errMsgs, "baudrate out of range");
    Utils::checkVar(parity, Device::Parity::None, Device::Parity::Space, errMsgs, "parity out of range");
    Utils::checkVar<uint_t>(dataBits, 5, 8, errMsgs, "dataBits out of range");
    Utils::checkVar(stopBits, Device::StopBits::One, Device::StopBits::Two, errMsgs, "stopBits out of range");
    Utils::checkVar<uint_t>(ahrsMode, 0, 1, errMsgs, "ahrsMode out of range");
    Utils::checkVar(orientationOffset.magnitude(), 0.99, 1.001, errMsgs, "orientationOffset quaternion is not normalised");
    Utils::checkVar(headingOffsetRad, -Math::pi2, Math::pi2, errMsgs, "headingOffsetRad out of range");
    Utils::checkVar(turnsAbout.magnitude(), 0.99, 1.001, errMsgs, "turnsAbout vector is not normalised");
    Utils::checkVar<uint_t>(multiEchoLimit, 0, 100, errMsgs, "multiEchoLimit out of range");
    Utils::checkVar<uint_t>(frequency, 50000, 700000, errMsgs, "frequency out of range");
    Utils::checkVar<uint_t>(txPulseWidthUs, 0, 500, errMsgs, "txPulseWidthUs out of range");
    Utils::checkVar<uint_t>(txPulseAmplitude, 0, 100, errMsgs, "txPulseAmplitude out of range");
    Utils::checkVar(echoAnalyseMode, EchoAnalyseMode::First, EchoAnalyseMode::Tracking, errMsgs, "echoAnalyseMode out of range");
    Utils::checkVar(xcThreasholdLow, 0.0, 1.0, errMsgs, "xcThreasholdLow out of range");
    Utils::checkVar(xcThreasholdHigh, 0.0, 1.0, errMsgs, "xcThreasholdHigh out of range");
    Utils::checkVar(energyThreashold, 0.0, 1.0, errMsgs, "energyThreashold out of range");
    Utils::checkVar(speedOfSound, 1000.0, 2000.0, errMsgs, "speedOfSound out of range");
    Utils::checkVar(maxRange - minRange, 0.001, 300.0, errMsgs, "(maxRange - minRange) out of range");
    Utils::checkVar(minRange, 0.0, 300.0, errMsgs, "minRange out of range");
    Utils::checkVar(maxRange, 0.0, 300.0, errMsgs, "maxRange out of range");
    Utils::checkVar(distanceOffset, -100.0, 100.0, errMsgs, "distanceOffset out of range");
    Utils::checkVar(anaMode, AnalogueOutMode::None, AnalogueOutMode::Current, errMsgs, "anaMode out of range");

    return errMsgs.empty();
}
//--------------------------------------------------------------------------------------------------
uint_t Isa500::Settings::serialise(uint8_t* buf, uint_t size) const
{
    if (size >= this->size)
    {
        uint8_t* start = buf;

        *buf++ = static_cast<uint8_t>(uartMode);
        Mem::pack32Bit(&buf, baudrate);
        *buf++ = static_cast<uint8_t>(parity);
        *buf++ = dataBits;
        *buf++ = static_cast<uint8_t>(stopBits);
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
        *buf++ = clrTurn.enable;
        *buf++ = clrTurn.size;
        Mem::memcpy(buf, &clrTurn.str[0], sizeof(clrTurn.str));
        buf += sizeof(clrTurn.str);
        *buf++ = setHeading2Mag.enable;
        *buf++ = setHeading2Mag.size;
        Mem::memcpy(buf, &setHeading2Mag.str[0], sizeof(setHeading2Mag.str));
        buf += sizeof(setHeading2Mag.str);
        *buf++ = multiEchoLimit;
        Mem::pack32Bit(&buf, frequency);
        Mem::pack16Bit(&buf, txPulseWidthUs);
        *buf++ = txPulseAmplitude;
        *buf++ = static_cast<uint8_t>(echoAnalyseMode);
        Mem::packFloat32(&buf, xcThreasholdLow);
        Mem::packFloat32(&buf, xcThreasholdHigh);
        Mem::packFloat32(&buf, energyThreashold);
        Mem::packFloat32(&buf, speedOfSound);
        Mem::packFloat32(&buf, minRange);
        Mem::packFloat32(&buf, maxRange);
        Mem::packFloat32(&buf, distanceOffset);
        *buf++ = (useTiltCorrection != 0) | ((useMaxValueOnNoReturn != 0) << 1);
        *buf++ = static_cast<uint8_t>(anaMode);
        Mem::packFloat32(&buf, aOutMinRange);
        Mem::packFloat32(&buf, aOutMaxRange);
        Mem::packFloat32(&buf, aOutMinVal);
        Mem::packFloat32(&buf, aOutMaxVal);

        *buf++ = strTrigger[0].strId;
        *buf++ = static_cast<uint8_t>(strTrigger[0].intervalEnabled);
        Mem::pack32Bit(&buf, strTrigger[0].intervalMs);
        *buf++ = static_cast<uint8_t>(strTrigger[0].triggerEnabled);
        *buf++ = static_cast<uint8_t>(strTrigger[0].triggerEdge);
        *buf++ = static_cast<uint8_t>(strTrigger[0].interrogation.enable);
        *buf++ = strTrigger[0].interrogation.size;
        Mem::memcpy(buf, &strTrigger[0].interrogation.str[0], sizeof(strTrigger[0].interrogation.str));
        buf += sizeof(strTrigger[0].interrogation.str);

        *buf++ = strTrigger[1].strId;
        *buf++ = static_cast<uint8_t>(strTrigger[1].intervalEnabled);
        Mem::pack32Bit(&buf, strTrigger[1].intervalMs);
        *buf++ = static_cast<uint8_t>(strTrigger[1].triggerEnabled);
        *buf++ = static_cast<uint8_t>(strTrigger[1].triggerEdge);
        *buf++ = static_cast<uint8_t>(strTrigger[1].interrogation.enable);
        *buf++ = strTrigger[1].interrogation.size;
        Mem::memcpy(buf, &strTrigger[1].interrogation.str[0], sizeof(strTrigger[1].interrogation.str));
        buf += sizeof(strTrigger[1].interrogation.str);

        return buf - start;
    }

    return 0;
}
//--------------------------------------------------------------------------------------------------
uint_t Isa500::Settings::deserialise(const uint8_t* data, uint_t size)
{
    if (size >= this->size)
    {
        const uint8_t* start = data;

        uartMode = static_cast<Device::UartMode>(*data++);
        baudrate = Mem::get32Bit(&data);
        parity = static_cast<Device::Parity>(*data++);
        dataBits = *data++;
        stopBits = static_cast<Device::StopBits>(*data++);
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
        clrTurn.enable = *data++;
        clrTurn.size = *data++;
        Mem::memcpy(&clrTurn.str[0], data, sizeof(clrTurn.str));
        data += sizeof(clrTurn.str);
        setHeading2Mag.enable = *data++;
        setHeading2Mag.size = *data++;
        Mem::memcpy(&setHeading2Mag.str[0], data, sizeof(setHeading2Mag.str));
        data += sizeof(setHeading2Mag.str);
        multiEchoLimit = *data++;
        frequency = Mem::get32Bit(&data);
        txPulseWidthUs = Mem::get16Bit(&data);
        txPulseAmplitude = *data++;
        echoAnalyseMode = static_cast<EchoAnalyseMode>(*data++);
        xcThreasholdLow = Mem::getFloat32(&data);
        xcThreasholdHigh = Mem::getFloat32(&data);
        energyThreashold = Mem::getFloat32(&data);
        speedOfSound = Mem::getFloat32(&data);
        minRange = Mem::getFloat32(&data);
        maxRange = Mem::getFloat32(&data);
        distanceOffset = Mem::getFloat32(&data);
        useTiltCorrection = *data & 0x01;
        useMaxValueOnNoReturn = *data & 0x02;
        data++;
        anaMode = static_cast<AnalogueOutMode>(*data++);
        aOutMinRange = Mem::getFloat32(&data);
        aOutMaxRange = Mem::getFloat32(&data);
        aOutMinVal = Mem::getFloat32(&data);
        aOutMaxVal = Mem::getFloat32(&data);

        strTrigger[0].strId = *data++;
        strTrigger[0].intervalEnabled = (bool_t)*data++;
        strTrigger[0].intervalMs = Mem::get32Bit(&data);
        strTrigger[0].triggerEnabled = (bool_t)*data++;
        strTrigger[0].triggerEdge = (bool_t)*data++;
        strTrigger[0].interrogation.enable = (bool_t)*data++;
        strTrigger[0].interrogation.size = *data++;
        Mem::memcpy(&strTrigger[0].interrogation.str[0], data, sizeof(strTrigger[0].interrogation.str));
        data += sizeof(strTrigger[0].interrogation.str);

        strTrigger[1].strId = *data++;
        strTrigger[1].intervalEnabled = (bool_t)*data++;
        strTrigger[1].intervalMs = Mem::get32Bit(&data);
        strTrigger[1].triggerEnabled = (bool_t)*data++;
        strTrigger[1].triggerEdge = (bool_t)*data++;
        strTrigger[1].interrogation.enable = (bool_t)*data++;
        strTrigger[1].interrogation.size = *data++;
        Mem::memcpy(&strTrigger[1].interrogation.str[0], data, sizeof(strTrigger[1].interrogation.str));
        data += sizeof(strTrigger[1].interrogation.str);

        return data - start;
    }

    return 0;
}
//--------------------------------------------------------------------------------------------------
bool_t Isa500::Settings::load(const XmlElementPtr& xml)
{
    bool_t ok = xml != nullptr;

    if (ok)
    {
        uartMode = StringUtils::toUartMode(xml->getString("uartMode", "rs232"));
        baudrate = static_cast<uint32_t>(xml->getUint("baudrate", 115200));
        parity = StringUtils::toUartParity(xml->getString("parity", "none"));
        dataBits = static_cast<uint8_t>(xml->getUint("dataBits", 8));
        stopBits = StringUtils::toUartStopBits(xml->getString("stopBits", "1"));
        ahrsMode = static_cast<uint8_t>(xml->getUint("ahrsMode", 0));
        headingOffsetRad = xml->getReal("headingOffset", 0.0);

        XmlElementPtr node = xml->findElement("orientationOffset");
        if (node)
        {
            orientationOffset.w = node->getReal("w", 0.0);
            orientationOffset.x = node->getReal("x", 0.0);
            orientationOffset.y = node->getReal("y", 0.0);
            orientationOffset.z = node->getReal("z", 0.0);
        }
        node = xml->findElement("turnsAbout");
        if (node)
        {
            turnsAbout.x = node->getReal("x", 0.0);
            turnsAbout.y = node->getReal("y", 0.0);
            turnsAbout.z = node->getReal("z", 0.0);
        }
        turnsAboutEarthFrame = xml->getBool("turnsAboutEarthFrame", true);
        clrTurn.enable = xml->getBool("clrTurnStrEnable", true);
        clrTurn.size = static_cast<uint8_t>(xml->getBytes("clrTurnStr",
            &clrTurn.str[0], sizeof(clrTurn.str)));
        setHeading2Mag.enable = xml->getBool("setHeading2MagStrEnable", true);
        setHeading2Mag.size = static_cast<uint8_t>(xml->getBytes("setHeading2MagStr",
            &setHeading2Mag.str[0], sizeof(setHeading2Mag.str)));

        multiEchoLimit = static_cast<uint8_t>(xml->getUint("multiEchoLimit", 10));
        frequency = static_cast<uint32_t>(xml->getUint("frequency", 500000));
        txPulseWidthUs = static_cast<uint16_t>(xml->getUint("txPulseWidthUs", 200));
        txPulseAmplitude = static_cast<uint8_t>(xml->getUint("txPulseAmplitude", 80));
        echoAnalyseMode = static_cast<EchoAnalyseMode>(xml->getUint("echoAnalyseMode", 0));
        xcThreasholdLow = xml->getReal("xcThreasholdLow", 0.4);
        xcThreasholdHigh = xml->getReal("xcThreasholdHigh", 0.5);
        energyThreashold = xml->getReal("energyThreashold", 0.0);
        speedOfSound = xml->getReal("speedOfSound", 1482.0);
        minRange = xml->getReal("minRange", 0.7);
        maxRange = xml->getReal("maxRange", 120.0);
        distanceOffset = xml->getReal("distanceOffset", 0.0);
        useTiltCorrection = xml->getBool("useTiltCorrection", false);
        useMaxValueOnNoReturn = xml->getBool("useMaxValueOnNoReturn", false);
        anaMode = static_cast<AnalogueOutMode>(xml->getUint("anaMode", 0));
        aOutMinRange = xml->getReal("aOutMinRange", 0.0);
        aOutMaxRange = xml->getReal("aOutMaxRange", 120.0);
        aOutMinVal = xml->getReal("aOutMinVal", 0.0);
        aOutMaxVal = xml->getReal("aOutMaxVal", 5.0);

        node = xml->findElement("pingString");
        if (node)
        {
            strTrigger[0].strId = static_cast<uint8_t>(node->getUint("id", 0));
            strTrigger[0].intervalEnabled = node->getBool("intervalEnabled", false);
            strTrigger[0].intervalMs = static_cast<uint32_t>(node->getUint("intervalMs", 500));
            strTrigger[0].triggerEnabled = node->getBool("triggerEnabled", false);
            strTrigger[0].triggerEdge = node->getBool("triggerEdge", false);
            strTrigger[0].interrogation.enable = node->getBool("interrogationEnabled", false);
            strTrigger[0].interrogation.size = static_cast<uint8_t>(node->getBytes("interrogationStr",
                &strTrigger[0].interrogation.str[0], sizeof(strTrigger[0].interrogation.str)));
        }

        node = xml->findElement("ahrsString");
        if (node)
        {
            strTrigger[1].strId = static_cast<uint8_t>(node->getUint("id", 0));
            strTrigger[1].intervalEnabled = node->getBool("intervalEnabled", false);
            strTrigger[1].intervalMs = static_cast<uint32_t>(node->getUint("intervalMs", 500));
            strTrigger[1].triggerEnabled = node->getBool("triggerEnabled", false);
            strTrigger[1].triggerEdge = node->getBool("triggerEdge", false);
            strTrigger[1].interrogation.enable = node->getBool("interrogationEnabled", false);
            strTrigger[1].interrogation.size = static_cast<uint8_t>(node->getBytes("interrogationStr",
                &strTrigger[1].interrogation.str[0], sizeof(strTrigger[1].interrogation.str)));
        }
    }
    return ok;
}
//--------------------------------------------------------------------------------------------------
void Isa500::Settings::save(XmlElementPtr& xml) const
{
    if (xml)
    {
        xml->addString("uartMode", StringUtils::uartModeToStr(uartMode));
        xml->addUint("baudrate", baudrate);
        xml->addString("uartParity", StringUtils::uartParityToStr(parity));
        xml->addUint("dataBits", dataBits);
        xml->addString("uartStopBits", StringUtils::uartStopBitsToStr(stopBits));
        xml->addUint("ahrsMode", ahrsMode);
        xml->addReal("headingOffset", headingOffsetRad, 3);

        XmlElementPtr node = xml->addElement("orientationOffset");
        node->addReal("w", orientationOffset.w, 6);
        node->addReal("x", orientationOffset.x, 6);
        node->addReal("y", orientationOffset.y, 6);
        node->addReal("z", orientationOffset.z, 6);

        node = xml->addElement("turnsAbout");
        node->addReal("x", turnsAbout.x, 6);
        node->addReal("y", turnsAbout.y, 6);
        node->addReal("z", turnsAbout.z, 6);

        xml->addBool("turnsAboutEarthFrame", turnsAboutEarthFrame);
        xml->addBool("clrTurnStrEnable", clrTurn.enable);
        xml->addBytes("clrTurnStr", &clrTurn.str[0], clrTurn.size);
        xml->addBool("setHeading2MagStrEnable", setHeading2Mag.enable);
        xml->addBytes("setHeading2MagStr", &setHeading2Mag.str[0], setHeading2Mag.size);

        xml->addUint("multiEchoLimit", multiEchoLimit);
        xml->addUint("frequency", frequency);
        xml->addUint("txPulseWidthUs", txPulseWidthUs);
        xml->addUint("txPulseAmplitude", txPulseAmplitude);
        xml->addUint("echoAnalyseMode", static_cast<uint_t>(echoAnalyseMode));
        xml->addReal("xcThreasholdLow", xcThreasholdLow, 4);
        xml->addReal("xcThreasholdHigh", xcThreasholdHigh, 4);
        xml->addReal("energyThreashold", energyThreashold, 4);
        xml->addReal("speedOfSound", speedOfSound, 3);
        xml->addReal("minRange", minRange, 3);
        xml->addReal("maxRange", maxRange, 3);
        xml->addReal("distanceOffset", energyThreashold, 3);
        xml->addBool("useTiltCorrection", useTiltCorrection);
        xml->addBool("useMaxValueOnNoReturn", useMaxValueOnNoReturn);
        xml->addUint("anaMode", static_cast<uint_t>(anaMode));
        xml->addReal("aOutMinRange", aOutMinRange, 3);
        xml->addReal("aOutMaxRange", aOutMaxRange, 3);
        xml->addReal("aOutMinVal", aOutMinVal, 3);
        xml->addReal("aOutMaxVal", aOutMaxVal, 3);

        node = xml->addElement("pingString");
        node->addUint("id", strTrigger[0].strId);
        node->addBool("intervalEnabled", strTrigger[0].intervalEnabled);
        node->addUint("intervalMs", strTrigger[0].intervalMs);
        node->addBool("interrogationEnabled", strTrigger[0].interrogation.enable);
        node->addBytes("interrogationStr", &strTrigger[0].interrogation.str[0], strTrigger[0].interrogation.size);

        node = xml->addElement("ahrsString");
        node->addUint("id", strTrigger[1].strId);
        node->addBool("intervalEnabled", strTrigger[1].intervalEnabled);
        node->addUint("intervalMs", strTrigger[1].intervalMs);
        node->addBool("interrogationEnabled", strTrigger[1].interrogation.enable);
        node->addBytes("interrogationStr", &strTrigger[1].interrogation.str[0], strTrigger[1].interrogation.size);
    }
}
//--------------------------------------------------------------------------------------------------
