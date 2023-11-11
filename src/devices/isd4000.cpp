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
    m_requestedRates.pressure = 100;
    m_requestedRates.ahrs = 100;
    m_requestedRates.gyro = 100;
    m_requestedRates.accel = 100;
    m_requestedRates.mag = 100;
    m_requestedRates.temperature = 200;

    Mem::memset(&m_pressureCal, 0, sizeof(PressureCal));
    Mem::memset(&m_temperatureCal, 0, sizeof(TemperatureCal));
    Mem::memset(&m_pressureSensorInfo, 0, sizeof(PressureSenorInfo));

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
void Isd4000::startLogging()
{
    Device::startLogging();
    logSettings();
}
//--------------------------------------------------------------------------------------------------
bool_t Isd4000::saveConfig(const std::string& fileName)
{
    bool_t ok = true;

    getScripts();

    if (m_onDepth.state == DataState::Valid && m_onAhrs.state == DataState::Valid && getCal())
    {
        XmlFile file;

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
                    node2->addAttribute("id", StringUtils::uintToStr(i + 1));
                    node2->addReal("measured", m_pressureCal.cal.calPoints[i].x, 6);
                    node2->addReal("actual", m_pressureCal.cal.calPoints[i].y, 6);
                }

                node = xml->addElement("verifyPoints");
                for (uint_t i = 0; i < m_pressureCal.cal.verifyPointsLength; i++)
                {
                    XmlElementPtr node2 = node->addElement("point");
                    node2->addAttribute("id", StringUtils::uintToStr(i + 1));
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
                    node2->addAttribute("id", StringUtils::uintToStr(i + 1));
                    node2->addReal("measured", m_temperatureCal.cal.calPoints[i].x, 6);
                    node2->addReal("actual", m_temperatureCal.cal.calPoints[i].y, 6);
                }

                node = xml->addElement("verifyPoints");
                for (uint_t i = 0; i < m_temperatureCal.cal.verifyPointsLength; i++)
                {
                    XmlElementPtr node2 = node->addElement("point");
                    node2->addAttribute("id", StringUtils::uintToStr(i + 1));
                    node2->addReal("measured", m_temperatureCal.cal.verifyPoints[i].x, 6);
                    node2->addReal("actual", m_temperatureCal.cal.verifyPoints[i].y, 6);
                }
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
bool_t Isd4000::loadConfig(const std::string& fileName, Device::Info* info, Settings* settings, DeviceScript* script0, DeviceScript* script1, AhrsCal* ahrsCal, PressureCal* pCal, TemperatureCal* tCal)
{
    bool_t ok = false;

    XmlFile file;
    if (file.open(fileName))
    {
        XmlElementPtr baseNode = file.findElementOnRoot("ISD4000");
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
                            if (i && (i - 1) < countof(pCal->cal.calPoints))
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
                            if (i && (i - 1) < countof(pCal->cal.verifyPoints))
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
                            if (i && (i - 1) < countof(tCal->cal.calPoints))
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
                            if (i && (i - 1) < countof(tCal->cal.verifyPoints))
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
    case Commands::GetPressureCal:
    {
        m_pressureCal.cal.year = Mem::get16Bit(&data);
        m_pressureCal.cal.month = *data++;
        m_pressureCal.cal.day = *data++;
        m_pressureCal.cal.calPointsLength = *data++;
        m_pressureCal.cal.verifyPointsLength = *data++;
        for (uint_t i = 0; i < countof(m_pressureCal.cal.calPoints); i++)
        {
            m_pressureCal.cal.calPoints[i].x = Mem::getFloat32(&data);
            m_pressureCal.cal.calPoints[i].y = Mem::getFloat32(&data);
        }
        for (uint_t i = 0; i < countof(m_pressureCal.cal.verifyPoints); i++)
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

        if (!m_saveConfigPath.empty() && m_scriptVars.state == DataState::Valid && m_onDepth.state == DataState::Valid && m_onAhrs.state == DataState::Valid && m_temperatureCal.state == DataState::Valid)
        {
            saveConfig(m_saveConfigPath);
            m_saveConfigPath.clear();
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
        for (uint_t i = 0; i < countof(m_temperatureCal.cal.calPoints); i++)
        {
            m_temperatureCal.cal.calPoints[i].x = Mem::getFloat32(&data);
            m_temperatureCal.cal.calPoints[i].y = Mem::getFloat32(&data);
        }
        for (uint_t i = 0; i < countof(m_temperatureCal.cal.verifyPoints); i++)
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

        if (!m_saveConfigPath.empty() && m_scriptVars.state == DataState::Valid && m_onDepth.state == DataState::Valid && m_onAhrs.state == DataState::Valid && m_pressureCal.state == DataState::Valid)
        {
            saveConfig(m_saveConfigPath);
            m_saveConfigPath.clear();
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
            if (!m_saveConfigPath.empty() && m_temperatureCal.state == DataState::Valid && m_pressureCal.state == DataState::Valid)
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
            Mem::memset(&m_pressureSensorInfo, 0, sizeof(m_pressureSensorInfo));
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
void Isd4000::logSettings()
{
    uint8_t data[Settings::size + 1];

    data[0] = static_cast<uint8_t>(Device::Commands::ReplyBit) | static_cast<uint8_t>(Commands::GetSettings);
    m_settings.serialise(&data[1], sizeof(data) - 1);

    log(&data[0], sizeof(data), static_cast<uint8_t>(LoggingDataType::packetData), false);
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
void Isd4000::setGyroCal(uint_t sensorNum, const Vector3* bias)
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
void Isd4000::setAccelCal(uint_t sensorNum, const Vector3& bias, const Matrix3x3& transform)
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
void Isd4000::setMagCal(uint_t sensorNum, const Vector3& bias, const Matrix3x3& transform)
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
    for (uint_t i = 0; i < countof(cert.calPoints); i++)
    {
        Mem::packFloat32(&buf, cert.calPoints[i].x);
        Mem::packFloat32(&buf, cert.calPoints[i].y);
    }
    for (uint_t i = 0; i < countof(cert.verifyPoints); i++)
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
    filterPressure = false;
    depthOffset = 0.0;
    pressureOffset = 0.0;
    latitude = 57.1;
    tareStr = { false, 3, {reinterpret_cast<const uint8_t*>("#t\r") } };
    unTareStr = { false, 3, {reinterpret_cast<const uint8_t*>("#u\r")} };
    strTrigger[0] = { 0, false, 500, { false, 3, reinterpret_cast<const uint8_t*>("#d\r") } };
    strTrigger[1] = { 0, false, 500, { false, 3, reinterpret_cast<const uint8_t*>("#o\r") } };
}
//--------------------------------------------------------------------------------------------------
bool_t Isd4000::Settings::check(std::vector<std::string>& errMsgs) const
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
        *buf++ = clrTurn.size;
        Mem::memcpy(buf, &clrTurn.str[0], sizeof(clrTurn.str));
        buf += sizeof(clrTurn.str);
        *buf++ = setHeading2Mag.enable;
        *buf++ = setHeading2Mag.size;
        Mem::memcpy(buf, &setHeading2Mag.str[0], sizeof(setHeading2Mag.str));
        buf += sizeof(setHeading2Mag.str);
        *buf++ = filterPressure;
        Mem::packFloat32(&buf, depthOffset);
        Mem::packFloat32(&buf, pressureOffset);
        Mem::packFloat32(&buf, latitude);
        *buf++ = tareStr.enable;
        *buf++ = tareStr.size;
        Mem::memcpy(buf, &tareStr.str[0], sizeof(tareStr.str));
        buf += sizeof(tareStr.str);
        *buf++ = unTareStr.enable;
        *buf++ = unTareStr.size;
        Mem::memcpy(buf, &unTareStr.str[0], sizeof(unTareStr.str));
        buf += sizeof(unTareStr.str);

        *buf++ = strTrigger[0].strId;
        *buf++ = static_cast<uint8_t>(strTrigger[0].intervalEnabled);
        Mem::pack32Bit(&buf, strTrigger[0].intervalMs);
        *buf++ = static_cast<uint8_t>(strTrigger[0].interrogation.enable);
        *buf++ = strTrigger[0].interrogation.size;
        Mem::memcpy(buf, &strTrigger[0].interrogation.str[0], sizeof(strTrigger[0].interrogation.str));
        buf += sizeof(strTrigger[0].interrogation.str);

        *buf++ = strTrigger[1].strId;
        *buf++ = static_cast<uint8_t>(strTrigger[1].intervalEnabled);
        Mem::pack32Bit(&buf, strTrigger[1].intervalMs);
        *buf++ = static_cast<uint8_t>(strTrigger[1].interrogation.enable);
        *buf++ = strTrigger[1].interrogation.size;
        Mem::memcpy(buf, &strTrigger[1].interrogation.str[0], sizeof(strTrigger[1].interrogation.str));
        buf += sizeof(strTrigger[1].interrogation.str);

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
        filterPressure = *data & 0x01;
        data++;
        depthOffset = Mem::getFloat32(&data);
        pressureOffset = Mem::getFloat32(&data);
        latitude = Mem::getFloat32(&data);

        tareStr.enable = *data++;
        tareStr.size = *data++;
        Mem::memcpy(&tareStr.str[0], data, sizeof(tareStr.str));
        data += sizeof(tareStr.str);
        unTareStr.enable = *data++;
        unTareStr.size = *data++;
        Mem::memcpy(&unTareStr.str[0], data, sizeof(unTareStr.str));
        data += sizeof(unTareStr.str);

        strTrigger[0].strId = *data++;
        strTrigger[0].intervalEnabled = static_cast<bool_t>(*data++);
        strTrigger[0].intervalMs = Mem::get32Bit(&data);
        strTrigger[0].interrogation.enable = static_cast<bool_t>(*data++);
        strTrigger[0].interrogation.size = *data++;
        Mem::memcpy(&strTrigger[0].interrogation.str[0], data, sizeof(strTrigger[0].interrogation.str));
        data += sizeof(strTrigger[0].interrogation.str);

        strTrigger[1].strId = *data++;
        strTrigger[1].intervalEnabled = static_cast<bool_t>(*data++);
        strTrigger[1].intervalMs = Mem::get32Bit(&data);
        strTrigger[1].interrogation.enable = static_cast<bool_t>(*data++);
        strTrigger[1].interrogation.size = *data++;
        Mem::memcpy(&strTrigger[1].interrogation.str[0], data, sizeof(strTrigger[1].interrogation.str));

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
        clrTurn.size = static_cast<uint8_t>(xml->getBytes("clrTurnStr",
            &clrTurn.str[0], sizeof(clrTurn.str)));
        setHeading2Mag.enable = xml->getBool("setHeading2MagStrEnable", true);
        setHeading2Mag.size = static_cast<uint8_t>(xml->getBytes("setHeading2MagStr",
            &setHeading2Mag.str[0], sizeof(setHeading2Mag.str)));

        filterPressure = xml->getBool("filterPressure", true);
        depthOffset = xml->getReal("depthOffset", 0.0);
        pressureOffset = xml->getReal("pressureOffset", 0.0);
        latitude = xml->getReal("latitude", 57.1);
        tareStr.enable = xml->getBool("tareStrEnable", true);
        tareStr.size = static_cast<uint8_t>(xml->getBytes("tareStr",
            &tareStr.str[0], sizeof(tareStr.str)));
        unTareStr.enable = xml->getBool("unTareStrEnable", true);
        unTareStr.size = static_cast<uint8_t>(xml->getBytes("unTareStr",
            &unTareStr.str[0], sizeof(unTareStr.str)));

        node = xml->findElement("depthString");
        if (node)
        {
            strTrigger[0].strId = static_cast<uint8_t>(node->getUint("id", 0));
            strTrigger[0].intervalEnabled = node->getBool("intervalEnabled", false);
            strTrigger[0].intervalMs = static_cast<uint32_t>(node->getUint("intervalMs", 500));
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
            strTrigger[1].interrogation.enable = node->getBool("interrogationEnabled", false);
            strTrigger[1].interrogation.size = static_cast<uint8_t>(node->getBytes("interrogationStr",
                &strTrigger[1].interrogation.str[0], sizeof(strTrigger[1].interrogation.str)));
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
        xml->addBytes("clrTurnStr", &clrTurn.str[0], clrTurn.size);
        xml->addBool("setHeading2MagStrEnable", setHeading2Mag.enable);
        xml->addBytes("setHeading2MagStr", &setHeading2Mag.str[0], setHeading2Mag.size);

        xml->addBool("filterPressure", filterPressure);
        xml->addReal("depthOffset", depthOffset, 6);
        xml->addReal("pressureOffset", pressureOffset, 6);
        xml->addReal("latitude", latitude, 6);
        xml->addBool("tareStrEnable", tareStr.enable);
        xml->addBytes("tareStr", &tareStr.str[0], tareStr.size);
        xml->addBool("unTareStrEnable", tareStr.enable);
        xml->addBytes("unTareStr", &unTareStr.str[0], unTareStr.size);

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
