//------------------------------------------ Includes ----------------------------------------------

#include "utils/xmlSettings.h"
#include "utils/stringUtils.h"

using namespace IslSdk;

//--------------------------------------------------------------------------------------------------
void XmlSettings::saveDeviceInfo(const Device::Info& info, XmlElementPtr& xml)
{
    if (xml)
    {
        xml->addAttribute("pid", StringUtils::uintToStr(static_cast<uint_t>(info.pid), 0));
        xml->addAttribute("sn", StringUtils::pnSnToStr(info.pn, info.sn));
        xml->addAttribute("firmwareVersion", StringUtils::bcdVersionToStr(info.firmwareVersionBcd));
        xml->addAttribute("build", StringUtils::uintToStr(info.firmwareBuildNum, 0));
        xml->addAttribute("config", StringUtils::uintToStr(info.config, 0));
        xml->addAttribute("mode", StringUtils::uintToStr(info.mode, 0));
        xml->addAttribute("status", StringUtils::uintToStr(info.status, 0));
    }
}
//--------------------------------------------------------------------------------------------------
bool_t XmlSettings::loadDeviceInfo(Device::Info& info, const XmlElementPtr& xml)
{
    bool_t ok = false;

    if (xml)
    {
        uint32_t pnSn = StringUtils::toPnSn(xml->getAttribute("sn"), ok);
        info.pn = pnSn >> 16;
        info.sn = pnSn & 0xffff;
        info.firmwareVersionBcd = StringUtils::toBcdVersion(xml->getAttribute("firmwareVersion"), ok);
        info.firmwareBuildNum = static_cast<uint16_t>(StringUtils::toUint(xml->getAttribute("build"), ok));
        info.pid = static_cast<Device::Pid>(StringUtils::toUint(xml->getAttribute("pid"), ok));
        info.config = static_cast<uint8_t>(StringUtils::toUint(xml->getAttribute("config"), ok));
        info.mode = static_cast<uint8_t>(StringUtils::toUint(xml->getAttribute("mode"), ok));
        info.status = static_cast<uint16_t>(StringUtils::toUint(xml->getAttribute("status"), ok));
        info.inUse = false;
    }
    return ok;
}
//--------------------------------------------------------------------------------------------------
void XmlSettings::saveBias(const Vector3& bias, XmlElementPtr& xml)
{
    if (xml)
    {
        XmlElementPtr node = xml->addElement("bias");
        node->addReal("x", bias.x, 6);
        node->addReal("y", bias.y, 6);
        node->addReal("z", bias.z, 6);
    }
}
//--------------------------------------------------------------------------------------------------
bool_t XmlSettings::loadBias(Vector3& bias, const XmlElementPtr& xml)
{
    bool_t ok = false;

    if (xml)
    {
        XmlElementPtr node = xml->findElement("bias");
        if (node)
        {
            Vector3 bias;
            bias.x = node->getReal("x", 0.0);
            bias.y = node->getReal("y", 0.0);
            bias.z = node->getReal("z", 0.0);
            ok = true;
        }
    }

    return ok;
}
//--------------------------------------------------------------------------------------------------
void XmlSettings::saveTransform(const Matrix3x3& transform, XmlElementPtr& xml)
{
    if (xml)
    {
        XmlElementPtr node = xml->addElement("transform");
        node->addReal("m00", transform.m[0][0], 6);
        node->addReal("m01", transform.m[0][1], 6);
        node->addReal("m02", transform.m[0][2], 6);
        node->addReal("m10", transform.m[1][0], 6);
        node->addReal("m11", transform.m[1][1], 6);
        node->addReal("m12", transform.m[1][2], 6);
        node->addReal("m20", transform.m[2][0], 6);
        node->addReal("m21", transform.m[2][1], 6);
        node->addReal("m22", transform.m[2][2], 6);
    }
}
//--------------------------------------------------------------------------------------------------
bool_t XmlSettings::loadTransform(Matrix3x3& transform, const XmlElementPtr& xml)
{
    bool_t ok = false;

    if (xml)
    {
        XmlElementPtr node = xml->findElement("transform");
        if (node)
        {
            transform.m[0][0] = node->getReal("m00", 0.0);
            transform.m[0][1] = node->getReal("m01", 0.0);
            transform.m[0][2] = node->getReal("m02", 0.0);
            transform.m[1][0] = node->getReal("m10", 0.0);
            transform.m[1][1] = node->getReal("m11", 0.0);
            transform.m[1][2] = node->getReal("m12", 0.0);
            transform.m[2][0] = node->getReal("m20", 0.0);
            transform.m[2][1] = node->getReal("m21", 0.0);
            transform.m[2][2] = node->getReal("m22", 0.0);
            ok = true;
        }
    }

    return ok;
}
//--------------------------------------------------------------------------------------------------
void XmlSettings::saveScript(const DeviceScript& script, XmlElementPtr& xml)
{
    if (xml)
    {
        xml->addString("name", script.name);
        xml->addString("code", script.code);
    }
}
//--------------------------------------------------------------------------------------------------
bool_t XmlSettings::loadScript(DeviceScript& script, const XmlElementPtr& xml)
{
    bool_t ok = false;

    if (xml)
    {
        script.name = xml->getString("name", "");
        script.code = xml->getString("code", "");
        script.state = DataState::Valid;
        ok = true;
    }
    return ok;
}
//--------------------------------------------------------------------------------------------------
