//------------------------------------------ Includes ----------------------------------------------

#include "ism3d.h"
#include "maths/maths.h"
#include "platform/debug.h"
#include "platform/mem.h"
#include "utils/stringUtils.h"
#include "utils/xmlSettings.h"
#include "utils/utils.h"

using namespace IslSdk;

//--------------------------------------------------------------------------------------------------
Ism3d::Ism3d(const Device::Info& info) : Device(info)
{
    m_requestedRates.ahrs = 0;
    m_requestedRates.gyro = 0;
    m_requestedRates.accel = 0;
    m_requestedRates.mag = 0;

    ahrs.onData.setSubscribersChangedCallback(this, &Ism3d::signalSubscribersChanged);
    gyro.onData.setSubscribersChangedCallback(this, &Ism3d::signalSubscribersChanged);
    accel.onData.setSubscribersChangedCallback(this, &Ism3d::signalSubscribersChanged);
    mag.onData.setSubscribersChangedCallback(this, &Ism3d::signalSubscribersChanged);
}
//--------------------------------------------------------------------------------------------------
Ism3d::~Ism3d()
{
}
//--------------------------------------------------------------------------------------------------
void Ism3d::setSensorRates(const SensorRates& rates)
{
    m_requestedRates = rates;

    if (m_connected)
    {
        SensorRates toSend = m_requestedRates;

        if (!ahrs.onData.hasSubscribers()) toSend.ahrs = 0;
        if (!gyro.onData.hasSubscribers()) toSend.gyro = 0;
        if (!accel.onData.hasSubscribers()) toSend.accel = 0;
        if (!mag.onData.hasSubscribers()) toSend.mag = 0;

        uint8_t data[17];
        uint8_t* buf = &data[0];

        *buf++ = static_cast<uint8_t>(Commands::SetSensorInterval);
        Mem::pack32Bit(&buf, toSend.ahrs);
        Mem::pack32Bit(&buf, toSend.gyro);
        Mem::pack32Bit(&buf, toSend.accel);
        Mem::pack32Bit(&buf, toSend.mag);

        enqueuePacket(&data[0], sizeof(data));
    }
}
//--------------------------------------------------------------------------------------------------
bool_t Ism3d::setSettings(const Settings& newSettings, bool_t save)
{
    uint8_t data[Settings::size + 2];
    std::vector<std::string> errMsgs;
    bool_t ok = newSettings.check(errMsgs);

    if (ok)
    {
        data[0] = static_cast<uint8_t>(Commands::SetSettings);
        data[1] = static_cast<uint8_t>(save);
        newSettings.serialise(&data[2], sizeof(data) - 2);

        if (newSettings.baudrate != m_settings.baudrate || newSettings.uartMode != m_settings.uartMode)
        {
            connectionSettingsUpdated(ConnectionMeta(newSettings.baudrate), newSettings.uartMode != Uart::Mode::Rs232);
        }
        m_settings = newSettings;
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
bool_t Ism3d::setAhrsScript(const std::string& name, const std::string& code)
{
    uint8_t data[1024];
    uint8_t* buf = &data[0];

    if ((4 + name.size() + code.size()) <= sizeof(data))
    {
        *buf++ = static_cast<uint8_t>(Commands::SetScript);
        *buf++ = static_cast<uint8_t>(0);
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
bool_t Ism3d::getScripts()
{
    if (m_scriptVars.state == DataState::Invalid)
    {
        m_scriptVars.state = DataState::Pending;
        getScriptVars();
    }

    if (m_onAhrs.state == DataState::Invalid)
    {
        m_onAhrs.state = DataState::Pending;
        getScript();
    }

    return m_scriptVars.state == DataState::Valid && m_onAhrs.state == DataState::Valid;
}

//--------------------------------------------------------------------------------------------------
bool_t Ism3d::startLogging()
{
    Device::startLogging();
    return logSettings();
}
//--------------------------------------------------------------------------------------------------
std::vector<std::string> Ism3d::getHardwareFaults()
{
    enum FaultFlags : uint16_t { BackupGyro = 1, Mag = 2, GyroAccel = 4 };
    std::vector<std::string> faults;

    if (info.status)
    {
        if (info.status & FaultFlags::BackupGyro)
        {
            faults.emplace_back("backup gyro");
        }
        if (info.status & FaultFlags::Mag)
        {
            faults.emplace_back("mag");
        }
        if (info.status & FaultFlags::GyroAccel)
        {
            faults.emplace_back("gyro/accel");
        }
    }
	return faults;
}
//--------------------------------------------------------------------------------------------------
bool_t Ism3d::saveConfig(const std::string& fileName)
{
    bool_t ok = true;

    getScripts();

    if (m_onAhrs.state == DataState::Valid)
    {
        XmlFile file;
        ok = makeXmlConfig(file);
        if (ok)
        {
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
std::string Ism3d::getConfigAsString()
{
    std::string xml;

    getScripts();

    if (m_onAhrs.state == DataState::Valid)
    {
        m_waitingForXmlConfig = false;
        XmlFile file;
        makeXmlConfig(file);
        xml = file.asString();
        onXmlConfig(*this, xml);
    }
    else
    {
        m_waitingForXmlConfig = true;
    }

    return xml;
}
//--------------------------------------------------------------------------------------------------
bool_t Ism3d::makeXmlConfig(XmlFile& file)
{
    XmlElementPtr rootXml = file.setRoot("ISM3D");
    if (rootXml)
    {
        XmlSettings::saveDeviceInfo(info, rootXml);

        XmlElementPtr xml = rootXml->addElement("settings");
        m_settings.save(xml);

        xml = rootXml->addElement("script0");
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

            node = xml->addElement("gyroBackup");
            XmlSettings::saveBias(gyroSec.bias, node);
            node = xml->addElement("accelBackup");
            XmlSettings::saveBias(accelSec.bias, node);
            XmlSettings::saveTransform(accelSec.transform, node);

        }
    }

    return rootXml != nullptr;
}
//--------------------------------------------------------------------------------------------------
bool_t Ism3d::loadConfig(const std::string& fileName, Device::Info* info, Settings* settings, DeviceScript* script, AhrsCal* ahrsCal)
{
    bool_t ok = false;

    XmlFile file;
    if (file.open(fileName))
    {
        XmlElementPtr baseNode = file.root();
        if (baseNode && baseNode->name == "ISM3D")
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

            if (script)
            {
                XmlElementPtr xml = baseNode->findElement("script0");
                ok &= XmlSettings::loadScript(*script, xml);
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

                    node = xml->findElement("gyroBackup");
                    ok &= XmlSettings::loadBias(ahrsCal->gyroBiasSec, node);

                    node = xml->findElement("accelBackup");
                    ok &= XmlSettings::loadBias(ahrsCal->accelBiasSec, node);
                    ok &= XmlSettings::loadTransform(ahrsCal->accelTransformSec, node);
                }
            }
        }
    }
    return ok;
}
//--------------------------------------------------------------------------------------------------
void Ism3d::connectionEvent(bool_t isConnected)
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
            if (!m_connectionDataSynced)
            {
                getStringNames();
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
bool_t Ism3d::newPacket(uint8_t command, const uint8_t* data, uint_t size)
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
            uint64_t timeUs = Mem::get48Bit(&data) + m_epochUs;
            real_t w = Mem::getFloat32(&data);
            real_t x = Mem::getFloat32(&data);
            real_t y = Mem::getFloat32(&data);
            real_t z = Mem::getFloat32(&data);
            Math::Quaternion q = Math::Quaternion(w, x, y, z);
            real_t magHeadingRad = Mem::getFloat32(&data);
            real_t turnsCount = Mem::getFloat32(&data);
            ahrs.onData(ahrs, timeUs, q, magHeadingRad, turnsCount);
        }

        if (sensorFlags & DataFlags::gyro)
        {
            Math::Vector3 gyroVec, gyroVecSec;
            gyroVec.x = Mem::getFloat32(&data);
            gyroVec.y = Mem::getFloat32(&data);
            gyroVec.z = Mem::getFloat32(&data);
            gyroVecSec.x = Mem::getFloat32(&data);
            gyroVecSec.y = Mem::getFloat32(&data);
            gyroVecSec.z = Mem::getFloat32(&data);
            gyro.onData(gyro, gyroVec);
            gyroSec.onData(gyroSec, gyroVecSec);
        }

        if (sensorFlags & DataFlags::accel)
        {
            Math::Vector3 accelVec, accelVecSec;
            accelVec.x = Mem::getFloat32(&data);
            accelVec.y = Mem::getFloat32(&data);
            accelVec.z = Mem::getFloat32(&data);
            accelVecSec.x = Mem::getFloat32(&data);
            accelVecSec.y = Mem::getFloat32(&data);
            accelVecSec.z = Mem::getFloat32(&data);
            accel.onData(accel, accelVec);
            accelSec.onData(accelSec, accelVecSec);
        }

        if (sensorFlags & DataFlags::mag)
        {
            Math::Vector3 magVec;
            magVec.x = Mem::getFloat32(&data);
            magVec.y = Mem::getFloat32(&data);
            magVec.z = Mem::getFloat32(&data);
            mag.onData(mag, magVec);
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
            logSettings();
        }
        onSettingsUpdated(*this, *data != 0);
        break;
    }
    case Commands::GetAhrsCal:
    {
        Math::Vector3 gyroVec, accelVec, magVec, gyroVecSec, accelVecSec;
        Math::Matrix3x3 accelMat, accelMatSec, magMat;

        gyroVec.x = Mem::getFloat32(&data);
        gyroVec.y = Mem::getFloat32(&data);
        gyroVec.z = Mem::getFloat32(&data);
        accelVec.x = Mem::getFloat32(&data);
        accelVec.y = Mem::getFloat32(&data);
        accelVec.z = Mem::getFloat32(&data);
        accelMat[0][0] = Mem::getFloat32(&data);
        accelMat[0][1] = Mem::getFloat32(&data);
        accelMat[0][2] = Mem::getFloat32(&data);
        accelMat[1][0] = Mem::getFloat32(&data);
        accelMat[1][1] = Mem::getFloat32(&data);
        accelMat[1][2] = Mem::getFloat32(&data);
        accelMat[2][0] = Mem::getFloat32(&data);
        accelMat[2][1] = Mem::getFloat32(&data);
        accelMat[2][2] = Mem::getFloat32(&data);
        gyroVecSec.x = Mem::getFloat32(&data);
        gyroVecSec.y = Mem::getFloat32(&data);
        gyroVecSec.z = Mem::getFloat32(&data);
        accelVecSec.x = Mem::getFloat32(&data);
        accelVecSec.y = Mem::getFloat32(&data);
        accelVecSec.z = Mem::getFloat32(&data);
        magVec.x = Mem::getFloat32(&data);
        magVec.y = Mem::getFloat32(&data);
        magVec.z = Mem::getFloat32(&data);
        accelMatSec[0][0] = Mem::getFloat32(&data);
        accelMatSec[0][1] = Mem::getFloat32(&data);
        accelMatSec[0][2] = Mem::getFloat32(&data);
        accelMatSec[1][0] = Mem::getFloat32(&data);
        accelMatSec[1][1] = Mem::getFloat32(&data);
        accelMatSec[1][2] = Mem::getFloat32(&data);
        accelMatSec[2][0] = Mem::getFloat32(&data);
        accelMatSec[2][1] = Mem::getFloat32(&data);
        accelMatSec[2][2] = Mem::getFloat32(&data);
        magMat[0][0] = Mem::getFloat32(&data);
        magMat[0][1] = Mem::getFloat32(&data);
        magMat[0][2] = Mem::getFloat32(&data);
        magMat[1][0] = Mem::getFloat32(&data);
        magMat[1][1] = Mem::getFloat32(&data);
        magMat[1][2] = Mem::getFloat32(&data);
        magMat[2][0] = Mem::getFloat32(&data);
        magMat[2][1] = Mem::getFloat32(&data);
        magMat[2][2] = Mem::getFloat32(&data);
        gyro.updateCalValues(gyroVec);
        gyroSec.updateCalValues(gyroVecSec);
        accel.updateCalValues(accelVec, accelMat);
        accelSec.updateCalValues(accelVecSec, accelMatSec);
        mag.updateCalValues(magVec, magMat);
        break;
    }
    case Commands::SetGyroCal:
    {
        if (*data == 0)
        {
            onError(*this, "Gyro cal failed to save to device");
        }
        else
        {
            getAhrsCal();
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
        if (size >= 48)
        {
            Math::Vector3 magVec;
            Math::Matrix3x3 magMat;
            magVec.x = Mem::getFloat32(&data);
            magVec.y = Mem::getFloat32(&data);
            magVec.z = Mem::getFloat32(&data);
            magMat[0][0] = Mem::getFloat32(&data);
            magMat[0][1] = Mem::getFloat32(&data);
            magMat[0][2] = Mem::getFloat32(&data);
            magMat[1][0] = Mem::getFloat32(&data);
            magMat[1][1] = Mem::getFloat32(&data);
            magMat[1][2] = Mem::getFloat32(&data);
            magMat[2][0] = Mem::getFloat32(&data);
            magMat[2][1] = Mem::getFloat32(&data);
            magMat[2][2] = Mem::getFloat32(&data);
            mag.updateCalValues(magVec, magMat);
        }
        else if (*data == 0)
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

        if (m_scriptVars.state == DataState::Valid && m_onAhrs.state == DataState::Valid)
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
            m_onAhrs.name = StringUtils::toStr(data, size);
            data += m_onAhrs.name.size() + 1;
            size -= m_onAhrs.name.size() + 1;
            m_onAhrs.code = StringUtils::toStr(data, size);
            m_onAhrs.state = DataState::Valid;
        }

        if (m_scriptVars.state == DataState::Valid && m_onAhrs.state == DataState::Valid)
        {
            if (!m_saveConfigPath.empty())
            {
                saveConfig(m_saveConfigPath);
                m_saveConfigPath.clear();
            }
            if (m_waitingForXmlConfig)
            {
                getConfigAsString();
            }
            onScriptDataReceived(*this);
        }
        break;
    }
    case Commands::SetScript:
    {
        if (data[1] == 0)
        {
            onError(*this, "Script " + StringUtils::toStr(data[0]) + " failed to save to device");
        }
        break;
    }
    
    default:
        break;
    }

    return shouldLog;
}
//--------------------------------------------------------------------------------------------------
void Ism3d::signalSubscribersChanged(uint_t subscriberCount)
{
    if (subscriberCount <= 1)
    {
        setSensorRates(m_requestedRates);
    }
}
//--------------------------------------------------------------------------------------------------
bool_t Ism3d::logSettings()
{
    uint8_t data[Settings::size + 1];

    data[0] = static_cast<uint8_t>(Device::Commands::ReplyBit) | static_cast<uint8_t>(Commands::GetSettings);
    m_settings.serialise(&data[1], sizeof(data) - 1);

    return log(&data[0], sizeof(data), static_cast<uint8_t>(LoggingDataType::packetData), false);
}
//--------------------------------------------------------------------------------------------------
void Ism3d::getData(uint32_t flags)
{
    uint8_t data[5];

    data[0] = static_cast<uint8_t>(Commands::GetSensorData);
    Mem::pack32Bit(&data[1], flags);
    enqueuePacket(&data[0], sizeof(data));
}
//--------------------------------------------------------------------------------------------------
void Ism3d::getSettings()
{
    uint8_t data = static_cast<uint8_t>(Commands::GetSettings);
    enqueuePacket(&data, sizeof(data));
}
//--------------------------------------------------------------------------------------------------
void Ism3d::getAhrsCal()
{
    uint8_t data = static_cast<uint8_t>(Commands::GetAhrsCal);
    enqueuePacket(&data, sizeof(data));
}
//--------------------------------------------------------------------------------------------------
void Ism3d::setGyroCal(uint_t sensorNum, const Math::Vector3* bias)
{
    uint8_t data[14];
    uint8_t* buf = &data[0];

    *buf++ = static_cast<uint8_t>(Commands::SetGyroCal);
    *buf++ = static_cast<uint8_t>(sensorNum);

    if (bias)
    {
        Mem::packFloat32(&buf, bias->x);
        Mem::packFloat32(&buf, bias->y);
        Mem::packFloat32(&buf, bias->z);
    }

    enqueuePacket(&data[0], buf - &data[0]);
}
//--------------------------------------------------------------------------------------------------
void Ism3d::setAccelCal(uint_t sensorNum, const Math::Vector3& bias, const Math::Matrix3x3& transform)
{
    uint8_t data[50];
    uint8_t* buf = &data[0];

    *buf++ = static_cast<uint8_t>(Commands::SetAccelCal);;
    *buf++ = static_cast<uint8_t>(sensorNum);
    Mem::packFloat32(&buf, bias.x);
    Mem::packFloat32(&buf, bias.y);
    Mem::packFloat32(&buf, bias.z);
    Mem::packFloat32(&buf, transform[0][0]);
    Mem::packFloat32(&buf, transform[0][1]);
    Mem::packFloat32(&buf, transform[0][2]);
    Mem::packFloat32(&buf, transform[1][0]);
    Mem::packFloat32(&buf, transform[1][1]);
    Mem::packFloat32(&buf, transform[1][2]);
    Mem::packFloat32(&buf, transform[2][0]);
    Mem::packFloat32(&buf, transform[2][1]);
    Mem::packFloat32(&buf, transform[2][2]);

    enqueuePacket(&data[0], sizeof(data));
}
//--------------------------------------------------------------------------------------------------
void Ism3d::setMagCal(uint_t sensorNum, const Math::Vector3& bias, const Math::Matrix3x3& transform, bool_t factory)
{
    uint8_t data[51];
    uint8_t* buf = &data[0];

    *buf++ = static_cast<uint8_t>(Commands::SetMagCal);
    *buf++ = 0;
    Mem::packFloat32(&buf, bias.x);
    Mem::packFloat32(&buf, bias.y);
    Mem::packFloat32(&buf, bias.z);
    Mem::packFloat32(&buf, transform[0][0]);
    Mem::packFloat32(&buf, transform[0][1]);
    Mem::packFloat32(&buf, transform[0][2]);
    Mem::packFloat32(&buf, transform[1][0]);
    Mem::packFloat32(&buf, transform[1][1]);
    Mem::packFloat32(&buf, transform[1][2]);
    Mem::packFloat32(&buf, transform[2][0]);
    Mem::packFloat32(&buf, transform[2][1]);
    Mem::packFloat32(&buf, transform[2][2]);

    if (factory)
	{
		*buf++ = 1;
	}

    enqueuePacket(&data[0], sizeof(data) - !factory);
}
//--------------------------------------------------------------------------------------------------
void Ism3d::loadFactoryMagCal()
{
    uint8_t data = static_cast<uint8_t>(Commands::SetMagCal);

    enqueuePacket(&data, sizeof(data));
}
//--------------------------------------------------------------------------------------------------
void Ism3d::setHeading(const real_t* angleInRadians)
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
void Ism3d::clearTurnsCount()
{
    uint8_t data = static_cast<uint8_t>(Commands::ClearTurnsCount);
    enqueuePacket(&data, sizeof(data));
}
//--------------------------------------------------------------------------------------------------
void Ism3d::getStringNames()
{
    uint8_t data[2];

    data[0] = static_cast<uint8_t>(Commands::GetStringNames);
    data[1] = static_cast<uint8_t>(0);
    enqueuePacket(&data[0], sizeof(data));
}
//--------------------------------------------------------------------------------------------------
void Ism3d::getScriptVars()
{
    uint8_t data = static_cast<uint8_t>(Commands::GetScriptVars);
    enqueuePacket(&data, sizeof(data));
}
//--------------------------------------------------------------------------------------------------
void Ism3d::getScript()
{
    uint8_t data[2];

    data[0] = static_cast<uint8_t>(Commands::GetScript);
    data[1] = static_cast<uint8_t>(0);
    enqueuePacket(&data[0], sizeof(data));
}

//--------------------------------------------------------------------------------------------------
bool_t Ism3d::setScript(const std::string& name, const std::string& code)
{
    if (setAhrsScript(name, code))
    {
        m_onAhrs.name = name;
        m_onAhrs.code = code;
        m_onAhrs.state = DataState::Valid;
        return true;
    }
    return false;
}
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
Ism3d::Settings::Settings()
{
    defaults();
}
//--------------------------------------------------------------------------------------------------
void Ism3d::Settings::defaults()
{
    uartMode = Uart::Mode::Rs232;
    baudrate = 9600;
    parity = Uart::Parity::None;
    dataBits = 8;
    stopBits = Uart::StopBits::One;
    ahrsMode = 0;
    orientationOffset = { 1,0,0,0 };
    headingOffsetRad = 0;
    turnsAbout = { 0,0,1 };
    turnsAboutEarthFrame = false;
    clrTurn = Device::CustomStr(false, "#c\\r");
    setHeading2Mag = Device::CustomStr(false, "#m\\r");
    ahrsStr = { 0, false, 500, Device::CustomStr(false, "#o\\r") };
}
//--------------------------------------------------------------------------------------------------
bool_t Ism3d::Settings::check(std::vector<std::string>& errMsgs) const
{
    Utils::checkVar(uartMode, Uart::Mode::Rs232, Uart::Mode::Rs485Terminated, errMsgs, "uartMode out of range");
    Utils::checkVar<uint_t>(baudrate, 300, 115200, errMsgs, "baudrate out of range");
    Utils::checkVar(parity, Uart::Parity::None, Uart::Parity::Space, errMsgs, "parity out of range");
    Utils::checkVar<uint_t>(dataBits, 5, 8, errMsgs, "dataBits out of range");
    Utils::checkVar(stopBits, Uart::StopBits::One, Uart::StopBits::Two, errMsgs, "stopBits out of range");
    Utils::checkVar<uint_t>(ahrsMode, 0, 1, errMsgs, "ahrsMode out of range");
    Utils::checkVar(orientationOffset.magnitude(), 0.99, 1.001, errMsgs, "orientationOffset quaternion is not normalised");
    Utils::checkVar(headingOffsetRad, -Math::pi2, Math::pi2, errMsgs, "headingOffsetRad out of range");
    Utils::checkVar(turnsAbout.magnitude(), 0.99, 1.001, errMsgs, "turnsAbout vector is not normalised");
    Utils::checkVar<uint_t>(clrTurn.str.size(), 0, Device::CustomStr::size, errMsgs, "clrTurn string too long");
    Utils::checkVar<uint_t>(setHeading2Mag.str.size(), 0, Device::CustomStr::size, errMsgs, "setHeading2Mag string too long");
    Utils::checkVar<uint_t>(ahrsStr.interrogation.str.size(), 0, Device::CustomStr::size, errMsgs, "ahrsStr string too long");

    return errMsgs.empty();
}
//--------------------------------------------------------------------------------------------------
uint_t Ism3d::Settings::serialise(uint8_t* buf, uint_t size) const
{
    if (size >= this->size)
    {;
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
        *buf = clrTurn.packStr(buf+1);
        buf += clrTurn.size + 1;
        *buf++ = setHeading2Mag.enable;
        *buf = setHeading2Mag.packStr(buf+1);
        buf += setHeading2Mag.size + 1;

        *buf++ = ahrsStr.strId;
        *buf++ = static_cast<uint8_t>(ahrsStr.intervalEnabled);
        Mem::pack32Bit(&buf, ahrsStr.intervalMs);
        *buf++ = static_cast<uint8_t>(ahrsStr.interrogation.enable);
        *buf = ahrsStr.interrogation.packStr(buf+1);
        buf += ahrsStr.interrogation.size + 1;

        return buf - start;
    }

    return 0;
}
//--------------------------------------------------------------------------------------------------
uint_t Ism3d::Settings::deserialise(const uint8_t* data, uint_t size)
{
    if (size >= this->size)
    {
        const uint8_t* start = data;

        uartMode = static_cast<Uart::Mode>(*data++);
        baudrate = Mem::get32Bit(&data);
        parity = static_cast<Uart::Parity>(*data++);
        dataBits = *data++;
        stopBits = static_cast<Uart::StopBits>(*data++);
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
        uint_t size = *data++;
        clrTurn.unPackStr(data, size);
        data += clrTurn.size;
        setHeading2Mag.enable = *data++;
        size = *data++;
        setHeading2Mag.unPackStr(data, size);
        data += setHeading2Mag.size;

        ahrsStr.strId = *data++;
        ahrsStr.intervalEnabled = (bool_t)*data++;
        ahrsStr.intervalMs = Mem::get32Bit(&data);
        ahrsStr.interrogation.enable = (bool_t)*data++;
        size = *data++;
        ahrsStr.interrogation.unPackStr(data, size);
        data += ahrsStr.interrogation.size;

        return data - start;
    }

    return 0;
}
//--------------------------------------------------------------------------------------------------
bool_t Ism3d::Settings::load(const XmlElementPtr& xml)
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
        clrTurn.str = xml->getString("clrTurnStr", "#c\\r");
        setHeading2Mag.enable = xml->getBool("setHeading2MagStrEnable", true);
        setHeading2Mag.str = xml->getString("setHeading2Mag", "#m\\r");

        node = xml->findElement("outputString");
        if (node)
        {
            ahrsStr.strId = static_cast<uint8_t>(node->getUint("id", 0));
            ahrsStr.intervalEnabled = node->getBool("intervalEnabled", false);
            ahrsStr.intervalMs = static_cast<uint32_t>(node->getUint("intervalMs", 500));
            ahrsStr.interrogation.enable = node->getBool("interrogationEnabled", false);
            ahrsStr.interrogation.enable = node->getBool("interrogationEnabled", false);
            ahrsStr.interrogation.str = xml->getString("interrogationStr", "#o\\r");
        }
    }
    return ok;
}
//--------------------------------------------------------------------------------------------------
void Ism3d::Settings::save(XmlElementPtr& xml) const
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
        xml->addString("clrTurnStr", clrTurn.str);
        xml->addBool("setHeading2MagStrEnable", setHeading2Mag.enable);
        xml->addString("setHeading2MagStr", setHeading2Mag.str);

        node = xml->addElement("outputString");
        node->addUint("id", ahrsStr.strId);
        node->addBool("intervalEnabled", ahrsStr.intervalEnabled);
        node->addUint("intervalMs", ahrsStr.intervalMs);
        node->addBool("interrogationEnabled", ahrsStr.interrogation.enable);
        node->addString("interrogationStr", ahrsStr.interrogation.str);
    }
}
//--------------------------------------------------------------------------------------------------
