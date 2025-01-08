//------------------------------------------ Includes ----------------------------------------------

#include "isd4000.h"
#include "maths/maths.h"
#include "platform/debug.h"
#include "platform/mem.h"
#include "utils/stringUtils.h"
#include "utils/xmlSettings.h"
#include "utils/utils.h"

using namespace IslSdk;

//--------------------------------------------------------------------------------------------------
Isd4000::Isd4000(const Device::Info& info) : Device(info)
{
    m_requestedRates.pressure = 0;
    m_requestedRates.ahrs = 0;
    m_requestedRates.gyro = 0;
    m_requestedRates.accel = 0;
    m_requestedRates.mag = 0;
    m_requestedRates.temperature = 0;

    ahrs.onData.setSubscribersChangedCallback(this, &Isd4000::signalSubscribersChanged);
    gyro.onData.setSubscribersChangedCallback(this, &Isd4000::signalSubscribersChanged);
    accel.onData.setSubscribersChangedCallback(this, &Isd4000::signalSubscribersChanged);
    mag.onData.setSubscribersChangedCallback(this, &Isd4000::signalSubscribersChanged);
}
//--------------------------------------------------------------------------------------------------
Isd4000::~Isd4000()
{
}
//--------------------------------------------------------------------------------------------------
void Isd4000::setSensorRates(const SensorRates& rates)
{
    m_requestedRates = rates;

    if (m_connected)
    {
        SensorRates toSend = m_requestedRates;

        if (!onPressure.hasSubscribers()) toSend.pressure = 0;
        if (!ahrs.onData.hasSubscribers()) toSend.ahrs = 0;
        if (!gyro.onData.hasSubscribers()) toSend.gyro = 0;
        if (!accel.onData.hasSubscribers()) toSend.accel = 0;
        if (!mag.onData.hasSubscribers()) toSend.mag = 0;
        if (!onTemperature.hasSubscribers()) toSend.temperature = 0;

        uint8_t data[25];
        uint8_t* buf = &data[0];

        *buf++ = static_cast<uint8_t>(Commands::SetSensorInterval);
        Mem::pack32Bit(&buf, toSend.pressure);
        Mem::pack32Bit(&buf, toSend.ahrs);
        Mem::pack32Bit(&buf, toSend.gyro);
        Mem::pack32Bit(&buf, toSend.accel);
        Mem::pack32Bit(&buf, toSend.mag);
        Mem::pack32Bit(&buf, toSend.temperature);

        enqueuePacket(&data[0], sizeof(data));
    }
}
//--------------------------------------------------------------------------------------------------
bool_t Isd4000::setSettings(const Settings& newSettings, bool_t save)
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
bool_t Isd4000::setDepthScript(const std::string& name, const std::string& code)
{
    if (setScript(0, name, code))
    {
        m_onDepth.name = name;
        m_onDepth.code = code;
        m_onDepth.state = DataState::Valid;
        return true;
    }
    return false;
}
//--------------------------------------------------------------------------------------------------
bool_t Isd4000::setAhrsScript(const std::string& name, const std::string& code)
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
bool_t Isd4000::getScripts()
{
    if (m_scriptVars.state == DataState::Invalid)
    {
        m_scriptVars.state = DataState::Pending;
        getScriptVars();
    }

    if (m_onDepth.state == DataState::Invalid)
    {
        m_onDepth.state = DataState::Pending;
        getScript(0);
    }

    if (m_onAhrs.state == DataState::Invalid)
    {
        m_onAhrs.state = DataState::Pending;
        getScript(1);
    }

    return m_scriptVars.state == DataState::Valid && m_onDepth.state == DataState::Valid && m_onAhrs.state == DataState::Valid;
}
//--------------------------------------------------------------------------------------------------
void Isd4000::setPressureCalCert(const CalCert& cert)
{
    setCalCert(Commands::SetPressureCal, cert);
    m_pressureCal.cal = cert;
    m_pressureCal.state = DataState::Valid;
}
//--------------------------------------------------------------------------------------------------
void Isd4000::setTemperatureCalCert(const CalCert& cert)
{
    setCalCert(Commands::SetTemperatureCal, cert);
    m_temperatureCal.cal = cert;
    m_temperatureCal.state = DataState::Valid;
}
//--------------------------------------------------------------------------------------------------
bool_t Isd4000::getCal(bool_t pressure, bool_t temperature)
{
    if (m_pressureCal.state == DataState::Invalid || pressure)
    {
        m_pressureCal.state = DataState::Pending;
        uint8_t data = static_cast<uint8_t>(Commands::GetPressureCal);
        enqueuePacket(&data, sizeof(data));
    }

    if (m_temperatureCal.state == DataState::Invalid || temperature)
    {
        m_temperatureCal.state = DataState::Pending;
        uint8_t data = static_cast<uint8_t>(Commands::GetTemperatureCal);
        enqueuePacket(&data, sizeof(data));
    }

    return m_pressureCal.state == DataState::Valid && m_temperatureCal.state == DataState::Valid;
}

//--------------------------------------------------------------------------------------------------
void Isd4000::measureNow()
{
    getData(DataFlags::pressure | DataFlags::temperature);
}
//--------------------------------------------------------------------------------------------------
bool_t Isd4000::startLogging()
{
    Device::startLogging();
    return logSettings();
}
//--------------------------------------------------------------------------------------------------
std::vector<std::string> Isd4000::getHardwareFaults()
{
    enum FaultFlags : uint16_t { Temperature = 1, Pressure = 2, GyroAccel = 4, Mag = 8 };
    std::vector<std::string> faults;

    if (info.status)
    {
        if (info.status & FaultFlags::Temperature)
        {
            faults.emplace_back("temperature sensor");
        }
        if (info.status & FaultFlags::Pressure)
        {
            faults.emplace_back("pressure sensor");
        }
        if (info.status & FaultFlags::GyroAccel)
        {
            faults.emplace_back("gyro/accel");
        }
        if (info.status & FaultFlags::Mag)
        {
            faults.emplace_back("mag");
        }
    }
    return faults;
}
//--------------------------------------------------------------------------------------------------
bool_t Isd4000::saveConfig(const std::string& fileName)
{
    bool_t ok = true;

    getScripts();

    if (m_onDepth.state == DataState::Valid && m_onAhrs.state == DataState::Valid && getCal())
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
std::string Isd4000::getConfigAsString()
{
    std::string xml;

    getScripts();

    if (m_onDepth.state == DataState::Valid && m_onAhrs.state == DataState::Valid && getCal())
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
bool_t Isd4000::makeXmlConfig(XmlFile& file)
{
    XmlElementPtr rootXml = file.setRoot("ISD4000");
    if (rootXml)
    {
        XmlSettings::saveDeviceInfo(info, rootXml);

        XmlElementPtr xml = rootXml->addElement("settings");
        m_settings.save(xml);

        xml = rootXml->addElement("script0");
        XmlSettings::saveScript(m_onDepth, xml);

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

        xml = rootXml->addElement("pressureCal");
        if (xml)
        {
            xml->addUint("year", m_pressureCal.cal.year);
            xml->addUint("month", m_pressureCal.cal.month);
            xml->addUint("day", m_pressureCal.cal.day);
            xml->addString("number", m_pressureCal.cal.number);
            xml->addString("organisation", m_pressureCal.cal.organisation);
            xml->addString("person", m_pressureCal.cal.person);
            xml->addString("equipment", m_pressureCal.cal.equipment);
            xml->addString("equipmentSn", m_pressureCal.cal.equipmentSn);
            xml->addString("notes", m_pressureCal.cal.notes);

            XmlElementPtr node = xml->addElement("calPoints");
            for (uint_t i = 0; i < m_pressureCal.cal.calPointsLength; i++)
            {
                XmlElementPtr node2 = node->addElement("point");
                node2->addAttribute("id", StringUtils::toStr(i + 1));
                node2->addReal("measured", m_pressureCal.cal.calPoints[i].x, 6);
                node2->addReal("actual", m_pressureCal.cal.calPoints[i].y, 6);
            }

            node = xml->addElement("verifyPoints");
            for (uint_t i = 0; i < m_pressureCal.cal.verifyPointsLength; i++)
            {
                XmlElementPtr node2 = node->addElement("point");
                node2->addAttribute("id", StringUtils::toStr(i + 1));
                node2->addReal("measured", m_pressureCal.cal.verifyPoints[i].x, 6);
                node2->addReal("actual", m_pressureCal.cal.verifyPoints[i].y, 6);
            }
        }

        xml = rootXml->addElement("temperatureCal");
        if (xml)
        {
            xml->addUint("year", m_temperatureCal.cal.year);
            xml->addUint("month", m_temperatureCal.cal.month);
            xml->addUint("day", m_temperatureCal.cal.day);
            xml->addString("number", m_temperatureCal.cal.number);
            xml->addString("organisation", m_temperatureCal.cal.organisation);
            xml->addString("person", m_temperatureCal.cal.person);
            xml->addString("equipment", m_temperatureCal.cal.equipment);
            xml->addString("equipmentSn", m_temperatureCal.cal.equipmentSn);
            xml->addString("notes", m_temperatureCal.cal.notes);

            XmlElementPtr node = xml->addElement("calPoints");
            for (uint_t i = 0; i < m_temperatureCal.cal.calPointsLength; i++)
            {
                XmlElementPtr node2 = node->addElement("point");
                node2->addAttribute("id", StringUtils::toStr(i + 1));
                node2->addReal("measured", m_temperatureCal.cal.calPoints[i].x, 6);
                node2->addReal("actual", m_temperatureCal.cal.calPoints[i].y, 6);
            }

            node = xml->addElement("verifyPoints");
            for (uint_t i = 0; i < m_temperatureCal.cal.verifyPointsLength; i++)
            {
                XmlElementPtr node2 = node->addElement("point");
                node2->addAttribute("id", StringUtils::toStr(i + 1));
                node2->addReal("measured", m_temperatureCal.cal.verifyPoints[i].x, 6);
                node2->addReal("actual", m_temperatureCal.cal.verifyPoints[i].y, 6);
            }
        }
    }

    return rootXml != nullptr;
}
//--------------------------------------------------------------------------------------------------
bool_t Isd4000::loadConfig(const std::string& fileName, Device::Info* info, Settings* settings, DeviceScript* script0, DeviceScript* script1, AhrsCal* ahrsCal, PressureCal* pCal, TemperatureCal* tCal)
{
    bool_t ok = false;

    XmlFile file;
    if (file.open(fileName))
    {
        XmlElementPtr baseNode = file.root();
        if (baseNode && baseNode->name == "ISD4000")
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

            XmlElementPtr xml = baseNode->findElement("pressureCal");
            if (xml && pCal)
            {
                pCal->cal.year = static_cast<uint16_t>(xml->getUint("year", 0));
                pCal->cal.month = static_cast<uint8_t>(xml->getUint("month", 0));
                pCal->cal.day = static_cast<uint8_t>(xml->getUint("day", 0));

                pCal->cal.number = xml->getString("number", "");
                pCal->cal.organisation = xml->getString("organisation", "");
                pCal->cal.person = xml->getString("person", "");
                pCal->cal.equipment = xml->getString("equipment", "");
                pCal->cal.equipmentSn = xml->getString("equipmentSn", "");
                pCal->cal.notes = xml->getString("notes", "");

                pCal->cal.calPointsLength = 0;
                XmlElementPtr node = xml->findElement("calPoints");
                if (node)
                {
                    for (const XmlElementPtr& point : node->elements)
                    {
                        if (point->name == "point")
                        {
                            uint_t i = StringUtils::toUint(point->getAttribute("id"), ok);
                            if (i && (i - 1) < pCal->cal.calPoints.size())
                            {
                                real_t x = point->getReal("measured", -1000.0);
                                real_t y = point->getReal("actual", -1000.0);

                                if (x > -999.0 && y > -999.0)
                                {
                                    pCal->cal.calPoints[i - 1].x = x;
                                    pCal->cal.calPoints[i - 1].y = y;
                                    pCal->cal.calPointsLength++;
                                }
                                else
                                {
                                    break;
                                }
                            }
                        }
                    }
                }

                pCal->cal.verifyPointsLength = 0;
                node = xml->findElement("verifyPoints");
                if (node)
                {
                    for (const XmlElementPtr& point : node->elements)
                    {
                        if (point->name == "point")
                        {
                            uint_t i = StringUtils::toUint(point->getAttribute("id"), ok);
                            if (i && (i - 1) < pCal->cal.verifyPoints.size())
                            {
                                real_t x = point->getReal("measured", -1000.0);
                                real_t y = point->getReal("actual", -1000.0);

                                if (x > -999.0 && y > -999.0)
                                {
                                    pCal->cal.verifyPoints[i - 1].x = x;
                                    pCal->cal.verifyPoints[i - 1].y = y;
                                    pCal->cal.verifyPointsLength++;
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

            xml = baseNode->findElement("temperatureCal");
            if (xml && tCal)
            {
                tCal->cal.year = static_cast<uint16_t>(xml->getUint("year", 0));
                tCal->cal.month = static_cast<uint8_t>(xml->getUint("month", 0));
                tCal->cal.day = static_cast<uint8_t>(xml->getUint("day", 0));

                tCal->cal.number = xml->getString("number", "");
                tCal->cal.organisation = xml->getString("organisation", "");
                tCal->cal.person = xml->getString("person", "");
                tCal->cal.equipment = xml->getString("equipment", "");
                tCal->cal.equipmentSn = xml->getString("equipmentSn", "");
                tCal->cal.notes = xml->getString("notes", "");

                tCal->cal.calPointsLength = 0;
                XmlElementPtr node = xml->findElement("calPoints");
                if (node)
                {
                    for (const XmlElementPtr& point : node->elements)
                    {
                        if (point->name == "point")
                        {
                            uint_t i = StringUtils::toUint(point->getAttribute("id"), ok);
                            if (i && (i - 1) < tCal->cal.calPoints.size())
                            {
                                real_t x = point->getReal("measured", -1000.0);
                                real_t y = point->getReal("actual", -1000.0);

                                if (x > -999.0 && y > -999.0)
                                {
                                    tCal->cal.calPoints[i - 1].x = x;
                                    tCal->cal.calPoints[i - 1].y = y;
                                    tCal->cal.calPointsLength++;
                                }
                                else
                                {
                                    break;
                                }
                            }
                        }
                    }
                }

                tCal->cal.verifyPointsLength = 0;
                node = xml->findElement("verifyPoints");
                if (node)
                {
                    for (const XmlElementPtr& point : node->elements)
                    {
                        if (point->name == "point")
                        {
                            uint_t i = StringUtils::toUint(point->getAttribute("id"), ok);
                            if (i && (i - 1) < tCal->cal.verifyPoints.size())
                            {
                                real_t x = point->getReal("measured", -1000.0);
                                real_t y = point->getReal("actual", -1000.0);

                                if (x > -999.0 && y > -999.0)
                                {
                                    tCal->cal.verifyPoints[i - 1].x = x;
                                    tCal->cal.verifyPoints[i - 1].y = y;
                                    tCal->cal.verifyPointsLength++;
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
    }

    return ok;
}
//--------------------------------------------------------------------------------------------------
void Isd4000::connectionEvent(bool_t isConnected)
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
                getStringNames(0);
                getStringNames(1);
                getPressureSensorInfo();
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
bool_t Isd4000::newPacket(uint8_t command, const uint8_t* data, uint_t size)
{
    bool_t shouldLog = false;

    switch (static_cast<Commands>(command))
    {
    case Commands::GetSensorData:
    {
        shouldLog = true;
        uint_t sensorFlags = Mem::get32Bit(&data);

        if (sensorFlags & DataFlags::pressure)
        {
            uint64_t timeUs = Mem::get48Bit(&data) + m_epochUs;
            real_t barRaw = Mem::getFloat32(&data);
            real_t bar = Mem::getFloat32(&data);
            real_t depthM = Mem::getFloat32(&data);
            onPressure(*this, timeUs, bar, depthM, barRaw);
        }

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
            Math::Vector3 gyroVec;
            gyroVec.x = Mem::getFloat32(&data);
            gyroVec.y = Mem::getFloat32(&data);
            gyroVec.z = Mem::getFloat32(&data);
            gyro.onData(gyro, gyroVec);
        }

        if (sensorFlags & DataFlags::accel)
        {
            Math::Vector3 accelVec;
            accelVec.x = Mem::getFloat32(&data);
            accelVec.y = Mem::getFloat32(&data);
            accelVec.z = Mem::getFloat32(&data);
            accel.onData(accel, accelVec);
        }

        if (sensorFlags & DataFlags::mag)
        {
            Math::Vector3 magVec;
            magVec.x = Mem::getFloat32(&data);
            magVec.y = Mem::getFloat32(&data);
            magVec.z = Mem::getFloat32(&data);
            mag.onData(mag, magVec);
        }

        if (sensorFlags & DataFlags::temperature)
        {
            real_t tempRawC = Mem::getFloat32(&data);
            real_t tempC = Mem::getFloat32(&data);
            onTemperature(*this, tempC, tempRawC);
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
        Math::Vector3 gyroVec, accelVec, magVec;
        Math::Matrix3x3 accelMat, magMat;
        gyroVec.x = Mem::getFloat32(&data);
        gyroVec.y = Mem::getFloat32(&data);
        gyroVec.z = Mem::getFloat32(&data);
        accelVec.x = Mem::getFloat32(&data);
        accelVec.y = Mem::getFloat32(&data);
        accelVec.z = Mem::getFloat32(&data);
        magVec.x = Mem::getFloat32(&data);
        magVec.y = Mem::getFloat32(&data);
        magVec.z = Mem::getFloat32(&data);
        accelMat[0][0] = Mem::getFloat32(&data);
        accelMat[0][1] = Mem::getFloat32(&data);
        accelMat[0][2] = Mem::getFloat32(&data);
        accelMat[1][0] = Mem::getFloat32(&data);
        accelMat[1][1] = Mem::getFloat32(&data);
        accelMat[1][2] = Mem::getFloat32(&data);
        accelMat[2][0] = Mem::getFloat32(&data);
        accelMat[2][1] = Mem::getFloat32(&data);
        accelMat[2][2] = Mem::getFloat32(&data);
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
        accel.updateCalValues(accelVec, accelMat);
        mag.updateCalValues(magVec, magMat);
        break;
    }
    case Commands::GetPressureCal:
    {
        m_pressureCal.cal.year = Mem::get16Bit(&data);
        m_pressureCal.cal.month = *data++;
        m_pressureCal.cal.day = *data++;
        m_pressureCal.cal.calPointsLength = *data++;
        m_pressureCal.cal.verifyPointsLength = *data++;
        for (uint_t i = 0; i < m_pressureCal.cal.calPoints.size(); i++)
        {
            m_pressureCal.cal.calPoints[i].x = Mem::getFloat32(&data);
            m_pressureCal.cal.calPoints[i].y = Mem::getFloat32(&data);
        }
        for (uint_t i = 0; i < m_pressureCal.cal.verifyPoints.size(); i++)
        {
            m_pressureCal.cal.verifyPoints[i].x = Mem::getFloat32(&data);
            m_pressureCal.cal.verifyPoints[i].y = Mem::getFloat32(&data);
        }

        m_pressureCal.cal.number = StringUtils::toStr(data, 32);
        data += 32;
        m_pressureCal.cal.organisation = StringUtils::toStr(data, 32);
        data += 32;
        m_pressureCal.cal.person = StringUtils::toStr(data, 32);
        data += 32;
        m_pressureCal.cal.equipment = StringUtils::toStr(data, 32);
        data += 32;
        m_pressureCal.cal.equipmentSn = StringUtils::toStr(data, 32);
        data += 32;
        m_pressureCal.cal.notes = StringUtils::toStr(data, 96);
        data += 96;

        m_pressureCal.state = DataState::Valid;

        if (m_scriptVars.state == DataState::Valid && m_onDepth.state == DataState::Valid && m_onAhrs.state == DataState::Valid && m_temperatureCal.state == DataState::Valid)
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
        }

        onPressureCalCert(*this, m_pressureCal);
        break;
    }
    case Commands::GetTemperatureCal:
    {
        m_temperatureCal.cal.year = Mem::get16Bit(&data);
        m_temperatureCal.cal.month = *data++;
        m_temperatureCal.cal.day = *data++;
        m_temperatureCal.cal.calPointsLength = *data++;
        m_temperatureCal.cal.verifyPointsLength = *data++;
        for (uint_t i = 0; i < m_temperatureCal.cal.calPoints.size(); i++)
        {
            m_temperatureCal.cal.calPoints[i].x = Mem::getFloat32(&data);
            m_temperatureCal.cal.calPoints[i].y = Mem::getFloat32(&data);
        }
        for (uint_t i = 0; i < m_temperatureCal.cal.verifyPoints.size(); i++)
        {
            m_temperatureCal.cal.verifyPoints[i].x = Mem::getFloat32(&data);
            m_temperatureCal.cal.verifyPoints[i].y = Mem::getFloat32(&data);
        }

        m_temperatureCal.cal.number = StringUtils::toStr(data, 32);
        data += 32;
        m_temperatureCal.cal.organisation = StringUtils::toStr(data, 32);
        data += 32;
        m_temperatureCal.cal.person = StringUtils::toStr(data, 32);
        data += 32;
        m_temperatureCal.cal.equipment = StringUtils::toStr(data, 32);
        data += 32;
        m_temperatureCal.cal.equipmentSn = StringUtils::toStr(data, 32);
        data += 32;
        m_temperatureCal.cal.notes = StringUtils::toStr(data, 96);
        data += 96;

        m_temperatureCal.adcOffset = (int32_t)Mem::get32Bit(&data);
        m_temperatureCal.state = DataState::Valid;

        if (m_scriptVars.state == DataState::Valid && m_onDepth.state == DataState::Valid && m_onAhrs.state == DataState::Valid && m_pressureCal.state == DataState::Valid)
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
        }

        onTemperatureCalCert(*this, m_temperatureCal);
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
            m_hardCodedDepthOutputStrings.clear();
            for (uint_t i = 0; i < count; i++)
            {
                std::string str = StringUtils::toStr(data, size);
                m_hardCodedDepthOutputStrings.emplace_back(str);
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

        if (m_scriptVars.state == DataState::Valid && m_onDepth.state == DataState::Valid && m_onAhrs.state == DataState::Valid)
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
            m_onDepth.name = StringUtils::toStr(data, size);
            data += m_onDepth.name.size() + 1;
            size -= m_onDepth.name.size() + 1;
            m_onDepth.code = StringUtils::toStr(data, size);
            m_onDepth.state = DataState::Valid;
        }
        else if (idx == 1)
        {
            m_onAhrs.name = StringUtils::toStr(data, size);
            data += m_onAhrs.name.size() + 1;
            size -= m_onAhrs.name.size() + 1;
            m_onAhrs.code = StringUtils::toStr(data, size);
            m_onAhrs.state = DataState::Valid;
        }

        if (m_scriptVars.state == DataState::Valid && m_onDepth.state == DataState::Valid && m_onAhrs.state == DataState::Valid)
        {
            if (m_temperatureCal.state == DataState::Valid && m_pressureCal.state == DataState::Valid)
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
    case Commands::SetCalTemperatureAdc:
    {
        break;
    }
    case Commands::SetPressureCal:
    {
        if (*data == 0)
        {
            onError(*this, "Pressure cal failed to save to device");
        }
        break;
    }
    case Commands::SetTemperatureCal:
    {
        if (*data == 0)
        {
            onError(*this, "Temperature cal failed to save to device");
        }
        break;
    }
    case Commands::GetPressureSensorInfo:
    {
        if (size >= 22)
        {
            
            m_pressureSensorInfo.minPressure = Mem::getFloat32(&data[14]);
            m_pressureSensorInfo.maxPressure = Mem::getFloat32(&data[18]);
        }
        else
        {
            
            m_pressureSensorInfo.minPressure = 0.0;
            m_pressureSensorInfo.maxPressure = 0.0;
        }
        break;
    }
    default:
        break;
    }

    return shouldLog;
}
//--------------------------------------------------------------------------------------------------
void Isd4000::signalSubscribersChanged(uint_t subscriberCount)
{
    if (subscriberCount <= 1)
    {
        setSensorRates(m_requestedRates);
    }
}
//--------------------------------------------------------------------------------------------------
bool_t Isd4000::logSettings()
{
    uint8_t data[Settings::size + 1];

    data[0] = static_cast<uint8_t>(Device::Commands::ReplyBit) | static_cast<uint8_t>(Commands::GetSettings);
    m_settings.serialise(&data[1], sizeof(data) - 1);

    return log(&data[0], sizeof(data), static_cast<uint8_t>(LoggingDataType::packetData), false);
}
//--------------------------------------------------------------------------------------------------
void Isd4000::getData(uint32_t flags)
{
    uint8_t data[5];

    data[0] = static_cast<uint8_t>(Commands::GetSensorData);
    Mem::pack32Bit(&data[1], flags);
    enqueuePacket(&data[0], sizeof(data));
}
//--------------------------------------------------------------------------------------------------
void Isd4000::getSettings()
{
    uint8_t data = static_cast<uint8_t>(Commands::GetSettings);
    enqueuePacket(&data, sizeof(data));
}
//--------------------------------------------------------------------------------------------------
void Isd4000::getAhrsCal()
{
    uint8_t data = static_cast<uint8_t>(Commands::GetAhrsCal);
    enqueuePacket(&data, sizeof(data));
}
//--------------------------------------------------------------------------------------------------
void Isd4000::setGyroCal(uint_t sensorNum, const Math::Vector3* bias)
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
void Isd4000::setAccelCal(uint_t sensorNum, const Math::Vector3& bias, const Math::Matrix3x3& transform)
{
    uint8_t data[49];
    uint8_t* buf = &data[0];

    *buf++ = static_cast<uint8_t>(Commands::SetAccelCal);;
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
void Isd4000::setMagCal(uint_t sensorNum, const Math::Vector3& bias, const Math::Matrix3x3& transform, bool_t factory)
{
    uint8_t data[50];
    uint8_t* buf = &data[0];

    *buf++ = static_cast<uint8_t>(Commands::SetMagCal);
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
void Isd4000::loadFactoryMagCal()
{
    uint8_t data = static_cast<uint8_t>(Commands::SetMagCal);

    enqueuePacket(&data, sizeof(data));
}
//--------------------------------------------------------------------------------------------------
void Isd4000::setHeading(const real_t* angleInRadians)
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
void Isd4000::clearTurnsCount()
{
    uint8_t data = static_cast<uint8_t>(Commands::ClearTurnsCount);
    enqueuePacket(&data, sizeof(data));
}
//--------------------------------------------------------------------------------------------------
void Isd4000::getStringNames(uint_t listId)
{
    uint8_t data[2];

    data[0] = static_cast<uint8_t>(Commands::GetStringNames);
    data[1] = static_cast<uint8_t>(listId);
    enqueuePacket(&data[0], sizeof(data));
}
//--------------------------------------------------------------------------------------------------
void Isd4000::getScriptVars()
{
    uint8_t data = static_cast<uint8_t>(Commands::GetScriptVars);
    enqueuePacket(&data, sizeof(data));
}
//--------------------------------------------------------------------------------------------------
void Isd4000::getScript(uint_t number)
{
    uint8_t data[2];

    data[0] = static_cast<uint8_t>(Commands::GetScript);
    data[1] = static_cast<uint8_t>(number);
    enqueuePacket(&data[0], sizeof(data));
}
//--------------------------------------------------------------------------------------------------
bool_t Isd4000::setScript(uint_t number, const std::string& name, const std::string& code)
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
void Isd4000::setCalCert(Commands command, const CalCert& cert)
{
    uint8_t data[423] = {};
    uint8_t* buf = &data[0];

    *buf++ = static_cast<uint8_t>(command);
    Mem::pack16Bit(&buf, cert.year);
    *buf++ = cert.month;
    *buf++ = cert.day;
    *buf++ = cert.calPointsLength;
    *buf++ = cert.verifyPointsLength;
    for (uint_t i = 0; i < cert.calPoints.size(); i++)
    {
        Mem::packFloat32(&buf, cert.calPoints[i].x);
        Mem::packFloat32(&buf, cert.calPoints[i].y);
    }
    for (uint_t i = 0; i < cert.verifyPoints.size(); i++)
    {
        Mem::packFloat32(&buf, cert.verifyPoints[i].x);
        Mem::packFloat32(&buf, cert.verifyPoints[i].y);
    }

    StringUtils::copyMax(cert.number, buf, 32);
    buf += 32;
    StringUtils::copyMax(cert.organisation, buf, 32);
    buf += 32;
    StringUtils::copyMax(cert.person, buf, 32);
    buf += 32;
    StringUtils::copyMax(cert.equipment, buf, 32);
    buf += 32;
    StringUtils::copyMax(cert.equipmentSn, buf, 32);
    buf += 32;
    StringUtils::copyMax(cert.notes, buf, 96);
    buf += 96;

    enqueuePacket(&data[0], (buf - &data[0]));
}
//--------------------------------------------------------------------------------------------------
void Isd4000::getPressureSensorInfo()
{
    uint8_t data = static_cast<uint8_t>(Commands::GetPressureSensorInfo);
    enqueuePacket(&data, sizeof(data));
}
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
Isd4000::Settings::Settings()
{
    defaults();
}
//--------------------------------------------------------------------------------------------------
void Isd4000::Settings::defaults()
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
    filterPressure = false;
    depthOffset = 0.0;
    pressureOffset = 0.0;
    latitude = 57.1;
    tareStr = Device::CustomStr(false, "#t\\r");
    unTareStr = Device::CustomStr(false, "#u\\r");
    depthStr = { 0, false, 500, Device::CustomStr(false, "#d\\r") };
    ahrsStr = { 0, false, 500, Device::CustomStr(false, "#o\\r") };
}
//--------------------------------------------------------------------------------------------------
bool_t Isd4000::Settings::check(std::vector<std::string>& errMsgs) const
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
    Utils::checkVar<uint_t>(tareStr.str.size(), 0, Device::CustomStr::size, errMsgs, "tareStr string too long");
	Utils::checkVar<uint_t>(unTareStr.str.size(), 0, Device::CustomStr::size, errMsgs, "unTareStr string too long");
    Utils::checkVar<uint_t>(depthStr.interrogation.str.size(), 0, Device::CustomStr::size, errMsgs, "depthStr string too long");
    Utils::checkVar<uint_t>(ahrsStr.interrogation.str.size(), 0, Device::CustomStr::size, errMsgs, "ahrsStr string too long");

    return errMsgs.empty();
}
//--------------------------------------------------------------------------------------------------
uint_t Isd4000::Settings::serialise(uint8_t* buf, uint_t size) const
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
        *buf = clrTurn.packStr(buf+1);
        buf += clrTurn.size + 1;
        *buf++ = setHeading2Mag.enable;
        *buf = setHeading2Mag.packStr(buf+1);
        buf += setHeading2Mag.size + 1;
        *buf++ = filterPressure;
        Mem::packFloat32(&buf, depthOffset);
        Mem::packFloat32(&buf, pressureOffset);
        Mem::packFloat32(&buf, latitude);
        *buf++ = tareStr.enable;
        *buf = tareStr.packStr(buf + 1);
        buf += tareStr.size + 1;
        *buf++ = unTareStr.enable;
        *buf = unTareStr.packStr(buf + 1);
        buf += unTareStr.size + 1;
 
        *buf++ = depthStr.strId;
        *buf++ = static_cast<uint8_t>(depthStr.intervalEnabled);
        Mem::pack32Bit(&buf, depthStr.intervalMs);
        *buf++ = static_cast<uint8_t>(depthStr.interrogation.enable);
        *buf = depthStr.interrogation.packStr(buf+1);
        buf += depthStr.interrogation.size + 1;

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
uint_t Isd4000::Settings::deserialise(const uint8_t* data, uint_t size)
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
        filterPressure = *data & 0x01;
        data++;
        depthOffset = Mem::getFloat32(&data);
        pressureOffset = Mem::getFloat32(&data);
        latitude = Mem::getFloat32(&data);
        tareStr.enable = *data++;
        size = *data++;
        tareStr.unPackStr(data, size);
        data += tareStr.size;
        unTareStr.enable = *data++;
        size = *data++;
        unTareStr.unPackStr(data, size);
        data += unTareStr.size;

        depthStr.strId = *data++;
        depthStr.intervalEnabled = static_cast<bool_t>(*data++);
        depthStr.intervalMs = Mem::get32Bit(&data);
        depthStr.interrogation.enable = static_cast<bool_t>(*data++);
        size = *data++;
        depthStr.interrogation.unPackStr(data, size);
        data += depthStr.interrogation.size;

        ahrsStr.strId = *data++;
        ahrsStr.intervalEnabled = static_cast<bool_t>(*data++);
        ahrsStr.intervalMs = Mem::get32Bit(&data);
        ahrsStr.interrogation.enable = static_cast<bool_t>(*data++);
        size = *data++;
        ahrsStr.interrogation.unPackStr(data, size);
        data += ahrsStr.interrogation.size;

        return data - start;
    }

    return 0;
}
//--------------------------------------------------------------------------------------------------
bool_t Isd4000::Settings::load(const XmlElementPtr& xml)
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
        filterPressure = xml->getBool("filterPressure", true);
        depthOffset = xml->getReal("depthOffset", 0.0);
        pressureOffset = xml->getReal("pressureOffset", 0.0);
        latitude = xml->getReal("latitude", 57.1);
        tareStr.enable = xml->getBool("tareStrEnable", true);
        tareStr.str = xml->getString("tareStr", "#t\\r");
        unTareStr.str = xml->getString("unTareStr", "#u\\r");

        node = xml->findElement("depthString");
        if (node)
        {
            depthStr.strId = static_cast<uint8_t>(node->getUint("id", 0));
            depthStr.intervalEnabled = node->getBool("intervalEnabled", false);
            depthStr.intervalMs = static_cast<uint32_t>(node->getUint("intervalMs", 500));
            depthStr.interrogation.enable = node->getBool("interrogationEnabled", false);
            depthStr.interrogation.enable = node->getBool("interrogationEnabled", false);
            depthStr.interrogation.str = xml->getString("interrogationStr", "#d\\r");
        }

        node = xml->findElement("ahrsString");
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
void Isd4000::Settings::save(XmlElementPtr& xml) const
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

        xml->addBool("filterPressure", filterPressure);
        xml->addReal("depthOffset", depthOffset, 6);
        xml->addReal("pressureOffset", pressureOffset, 6);
        xml->addReal("latitude", latitude, 6);
        xml->addBool("tareStrEnable", tareStr.enable);
        xml->addString("tareStr", tareStr.str);
        xml->addBool("unTareStrEnable", tareStr.enable);
        xml->addString("unTareStr", unTareStr.str);
        node = xml->addElement("pingString");
        node->addUint("id", depthStr.strId);
        node->addBool("intervalEnabled", depthStr.intervalEnabled);
        node->addUint("intervalMs", depthStr.intervalMs);
        node->addBool("interrogationEnabled", depthStr.interrogation.enable);
        node->addString("interrogationStr", depthStr.interrogation.str);

        node = xml->addElement("ahrsString");
        node->addUint("id", ahrsStr.strId);
        node->addBool("intervalEnabled", ahrsStr.intervalEnabled);
        node->addUint("intervalMs", ahrsStr.intervalMs);
        node->addBool("interrogationEnabled", ahrsStr.interrogation.enable);
        node->addString("interrogationStr", ahrsStr.interrogation.str);
    }
}
//--------------------------------------------------------------------------------------------------
