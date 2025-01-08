//------------------------------------------ Includes ----------------------------------------------

#include "utils/stringUtils.h"
#include "maths/maths.h"
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <sstream>

using namespace IslSdk;
using namespace StringUtils;

//--------------------------------------------------------------------------------------------------

uint_t parseDigits(const char* str, uint_t& digits);

//--------------------------------------------------------------------------------------------------
std::string StringUtils::toStr(const uint8_t* str, uint_t maxSize)
{
    std::string s;

    if (str && *str && maxSize)
    {
        uint_t i = 0;
        while (str[i] && maxSize)
        {
            maxSize--;
            i++;
        }
        s.assign(reinterpret_cast<const char*>(str), i);
    }
    return s;
}
//--------------------------------------------------------------------------------------------------
std::string StringUtils::toStr(uint16_t number, uint_t leadingZeros)
{
    return toStr(static_cast<uint_t>(number), leadingZeros);
}
//--------------------------------------------------------------------------------------------------
std::string StringUtils::toStr(uint32_t number, uint_t leadingZeros)
{
    return toStr(static_cast<uint_t>(number), leadingZeros);
}
//--------------------------------------------------------------------------------------------------
std::string StringUtils::toStr(uint_t number, uint_t leadingZeros)
{
    std::ostringstream oss;
    oss << std::setw(leadingZeros) << std::setfill('0') << number;
    return oss.str();
}
//--------------------------------------------------------------------------------------------------
std::string StringUtils::toStr(int16_t number, uint_t leadingZeros, bool_t forceSign)
{
    return toStr(static_cast<int_t>(number), leadingZeros, forceSign);
}
//--------------------------------------------------------------------------------------------------
std::string StringUtils::toStr(int32_t number, uint_t leadingZeros, bool_t forceSign)
{
    return toStr(static_cast<int_t>(number), leadingZeros, forceSign);
}
//--------------------------------------------------------------------------------------------------
std::string StringUtils::toStr(int_t number, uint_t leadingZeros, bool_t forceSign)
{
    std::ostringstream oss;
    oss << (forceSign ? std::showpos : std::noshowpos) << std::setw(leadingZeros) << std::setfill('0') << std::internal << number;
    return oss.str();
}
//--------------------------------------------------------------------------------------------------
std::string StringUtils::toStr(real_t number, uint_t leadingZeros, uint_t precision, uint8_t format, bool_t forceSign)
{
    if (std::isnan(number))
    {
        return "nan";
    }

    if (!std::isfinite(number))
    {
        return "inf";
    }

    std::ostringstream oss;
    oss << std::setprecision(precision);

    if (forceSign)
    {
        oss << std::showpos;
    }

    leadingZeros = leadingZeros + precision + (precision != 0);

    if (format == 'e' || format == 'E')
    {
        leadingZeros += leadingZeros ? 4 : 0;
        oss << std::setw(leadingZeros) << std::setfill('0') << std::internal << std::scientific;
    }
    else
    {
        oss << std::setw(leadingZeros) << std::setfill('0') << std::internal << std::fixed;
    }

    oss << number;

    return oss.str();
}
//--------------------------------------------------------------------------------------------------
std::string StringUtils::toHexStr(uint_t number, uint_t leadingZeros, bool_t useCaps)
{
    std::ostringstream oss;
    oss << (useCaps ? std::uppercase : std::nouppercase) << std::setw(leadingZeros) << std::setfill('0') << std::hex << number;
    return oss.str();
}
//--------------------------------------------------------------------------------------------------
std::string StringUtils::pidToStr(Device::Pid pid)
{
    switch (pid)
    {
    case IslSdk::Device::Pid::Isa500v1:
        return "ISA500v1";
    case IslSdk::Device::Pid::Isd4000v1:
        return "ISD4000v1";
    case IslSdk::Device::Pid::Ism3dv1:
        return "ISM3Dv1";
    case IslSdk::Device::Pid::Iss360v1:
        return "ISS360v1";
    case IslSdk::Device::Pid::Isa500:
        return "ISA500";
    case IslSdk::Device::Pid::Isd4000:
        return "ISD4000";
    case IslSdk::Device::Pid::Ism3d:
        return "ISM3D";
    case IslSdk::Device::Pid::Sonar:
        return "Sonar";
    case IslSdk::Device::Pid::MultiPcp:
        return "MultiPcp";
        
    default:
        break;
    }

    return "Unknown";
}
//--------------------------------------------------------------------------------------------------
std::string StringUtils::pnSnToStr(uint16_t pn, uint16_t sn)
{
    std::string str;

    str += toStr(pn, 4);
    str += '.';
    str += toStr(sn, 4);

    return str;
}
//--------------------------------------------------------------------------------------------------
std::string StringUtils::bcdVersionToStr(uint16_t bcdVersion)
{
    std::string str;

    if (((bcdVersion >> 12) & 0x0f))
    {
        str += ((bcdVersion >> 12) & 0x0f) | '0';
        str += '.';
        str += ((bcdVersion >> 8) & 0x0f) | '0';
        str += '.';
        str += ((bcdVersion >> 4) & 0x0f) | '0';
        str += '.';
        str += (bcdVersion & 0x0f) | '0';
    }
    else if (((bcdVersion >> 8) & 0x0f))
    {
        str += ((bcdVersion >> 8) & 0x0f) | '0';
        str += '.';
        str += ((bcdVersion >> 4) & 0x0f) | '0';
        str += '.';
        str += (bcdVersion & 0x0f) | '0';
    }
    else if (((bcdVersion >> 4) & 0x0f))
    {
        str += ((bcdVersion >> 4) & 0x0f) | '0';
        str += '.';
        str += (bcdVersion & 0x0f) | '0';
    }
    else
    {
        str += (bcdVersion & 0x0f) | '0';
    }

    return str;
}
//--------------------------------------------------------------------------------------------------
std::string StringUtils::uartModeToStr(Uart::Mode mode)
{
    switch (mode)
    {
    case Uart::Mode::Rs232:
        return "RS232";
    case Uart::Mode::Rs485:
        return "RS485";
    case Uart::Mode::Rs485Terminated:
        return "RS485T";
    default:
        break;
    }

    return "";
}
//--------------------------------------------------------------------------------------------------
std::string StringUtils::uartParityToStr(Uart::Parity parity)
{
    switch (parity)
    {
    case Uart::Parity::None:
        return "NONE";
    case Uart::Parity::Odd:
        return "ODD";
    case Uart::Parity::Even:
        return "EVEN";
    case Uart::Parity::Mark:
        return "MARK";
    case Uart::Parity::Space:
        return "SPACE";
    default:
        break;
    }

    return "";
}
//--------------------------------------------------------------------------------------------------
std::string StringUtils::uartStopBitsToStr(Uart::StopBits stopBits)
{
    switch (stopBits)
    {
    case Uart::StopBits::One:
        return "1";
    case Uart::StopBits::OneAndHalf:
        return "1.5";
    case Uart::StopBits::Two:
        return "2";
    default:
        break;
    }

    return "";
}
//--------------------------------------------------------------------------------------------------
std::string StringUtils::phyPortModeToStr(Device::PhyPortMode portMode)
{
    switch (portMode)
    {
    case Device::PhyPortMode::Auto:
        return "PHY_AUTO";
    case Device::PhyPortMode::Base10TxHalf:
        return "10_BASE_H";
    case Device::PhyPortMode::Base10TxFull:
        return "10_BASE_F";
    case Device::PhyPortMode::Base100TxHalf:
        return "100_BASE_H";
    case Device::PhyPortMode::Base100TxFull:
        return "100_BASE_F";
    default:
        break;
    }

    return "";
}
//--------------------------------------------------------------------------------------------------
std::string StringUtils::phyMdixModeToStr(Device::PhyMdixMode mdixMode)
{
    switch (mdixMode)
    {
    case Device::PhyMdixMode::Normal:
        return "MDIX_NORMAL";
    case Device::PhyMdixMode::Swapped:
        return "MDIX_SWAP";
    case Device::PhyMdixMode::Auto:
        return "MDIX_AUTO";
    default:
        break;
    }

    return "";
}
//--------------------------------------------------------------------------------------------------
std::string StringUtils::macAddressToStr(const uint8_t(&macAddress)[6])
{
    std::string str;

    str += toHexStr(macAddress[0], 2, true);
    str += ':';
    str += toHexStr(macAddress[1], 2, true);
    str += ':';
    str += toHexStr(macAddress[2], 2, true);
    str += ':';
    str += toHexStr(macAddress[3], 2, true);
    str += ':';
    str += toHexStr(macAddress[4], 2, true);
    str += ':';
    str += toHexStr(macAddress[5], 2, true);

    return str;
}
//--------------------------------------------------------------------------------------------------
std::string StringUtils::ipToStr(uint32_t ip)
{
    std::string str;

    str += toStr((ip >> 0) & 0xff, 0);
    str += '.';
    str += toStr((ip >> 8) & 0xff, 0);
    str += '.';
    str += toStr((ip >> 16) & 0xff, 0);
    str += '.';
    str += toStr((ip >> 24) & 0xff, 0);

    return str;
}
//--------------------------------------------------------------------------------------------------
uint_t StringUtils::toUint(const std::string& str, bool_t& error)
{
    uint_t index = 0;
    return toUint(str, index, 0, error);
}
//--------------------------------------------------------------------------------------------------
uint_t StringUtils::toUint(const std::string& str, uint_t& index, uint_t digits, bool_t& error)
{
    uint_t result = 0;

    if (!str.empty())
    {
        if (str[index] == '+')
        {
            index++;
        }

        result = parseDigits(&str[index], digits);
        index += digits;
    }

    error |= digits == 0 || str.empty();

    return result;
}
//--------------------------------------------------------------------------------------------------
int_t StringUtils::toInt(const std::string& str, bool_t& error)
{
    uint_t index = 0;
    return toInt(str, index, 0, error);
}
//--------------------------------------------------------------------------------------------------
int_t StringUtils::toInt(const std::string& str, uint_t& index, uint_t digits, bool_t& error)
{
    int_t result = 0;
    int_t sign = 1;

    if (!str.empty())
    {
        if (str[index] == '-' || str[index] == '+')
        {
            if (str[index] == '-')
            {
                sign = -1;
            }
            index++;
        }

        result = static_cast<int_t>(parseDigits(&str[index], digits)) * sign;
        index += digits;
    }
    error |= digits == 0 || str.empty();

    return result;
}
//--------------------------------------------------------------------------------------------------
real_t StringUtils::toReal(const std::string& str, bool_t& error)
{
    uint_t index = 0;
    return StringUtils::toReal(str, index, error);
}
//--------------------------------------------------------------------------------------------------
real_t StringUtils::toReal(const std::string& str, uint_t& index, bool_t& error)
{
    real_t result = 0.0;
    uint_t count = 0;
    char* end;

    if (!str.empty())
    {
        if (sizeof(real_t) == sizeof(float))
        {
            result = std::strtof(&str[index], &end);
        }
        else
        {
            result = std::strtod(&str[index], &end);
        }
        count = end - &str[index];
        index += count;
    }
    error |= count == 0;

    return result;
}
//--------------------------------------------------------------------------------------------------
uint_t StringUtils::hexToUint(const std::string& str, bool_t& error)
{
    uint_t index = 0;
    return StringUtils::hexToUint(str, index, 0, error);
}
//--------------------------------------------------------------------------------------------------
uint_t StringUtils::hexToUint(const std::string& str, uint_t& index, uint_t digits, bool_t& error)
{
    uint_t nibble = 0;
    uint_t num = 0;
    uint_t count = 0;
    uint_t overflow = sizeof(uint_t) * 2;
    int_t chars = digits;

    if (chars == 0)
    {
        chars = -1;
    }

    while (chars && count <= overflow && index < str.length())
    {
        if (str[index] >= '0' && str[index] <= '9')
        {
            nibble = str[index] - '0';
        }
        else if (str[index] >= 'A' && str[index] <= 'F')
        {
            nibble = str[index] - 'A' + 10;
        }
        else if (str[index] >= 'a' && str[index] <= 'f')
        {
            nibble = str[index] - 'a' + 10;
        }
        else
        {
            break;
        }

        num <<= 4;
        num |= nibble;
        count++;
        index++;
        chars--;
    }

    if (count == 0 || count == overflow || chars > 0)
    {
        error = true;
        index -= count;
        num = 0;
    }

    return num;
}
//--------------------------------------------------------------------------------------------------
Device::Pid StringUtils::toPid(const std::string& str)
{
    if (compareNoCase(str, "isa500v1"))
    {
        return Device::Pid::Isa500v1;
    }
    else if (compareNoCase(str, "isd4000v1"))
    {
        return Device::Pid::Isd4000v1;
    }
    else if (compareNoCase(str, "ism3dv1"))
    {
        return Device::Pid::Ism3dv1;
    }
    else if (compareNoCase(str, "iss360v1"))
    {
        return Device::Pid::Iss360v1;
    }
    else if (compareNoCase(str, "isa500"))
    {
        return Device::Pid::Isa500;
    }
    else if (compareNoCase(str, "isd4000"))
    {
        return Device::Pid::Isd4000;
    }
    else if (compareNoCase(str, "ism3d"))
    {
        return Device::Pid::Ism3d;
    }
    else if (compareNoCase(str, "iss360"))
    {
        return Device::Pid::Sonar;
    }
    else if (compareNoCase(str, "multipcp"))
    {
        return Device::Pid::MultiPcp;
    }

    return Device::Pid::Unknown;
}
//--------------------------------------------------------------------------------------------------
uint32_t StringUtils::toPnSn(const std::string& str, bool_t& error)
{
    uint_t pn = 0;
    uint_t sn = 0;
    uint_t index = 0;

    pn = toUint(str, index, 0, error);
    if (!error && (str[index] == '.' || str[index] == '-'))
    {
        index++;
        sn = toUint(str, index, 0, error);
    }

    return static_cast<uint32_t>((pn << 16) | sn);
}
//--------------------------------------------------------------------------------------------------
uint16_t StringUtils::toBcdVersion(const std::string& str, bool_t& error)
{
    uint16_t version = 0x0000;
    bool_t err = false;
    uint_t index = 0;

    while (index < str.length())
    {
        version <<= 4;
        uint_t digit = toUint(str, index, 0, err);
        if (!err && digit < 16)
        {
            version |= digit;
        }
        else
        {
            error |= index == 0;
            return 0;
        }

        if (str[index] == '.')
        {
            index++;
        }
    }

    return version;
}
//--------------------------------------------------------------------------------------------------
Uart::Mode StringUtils::toUartMode(const std::string& str)
{
    if (compareNoCase(str, "rs232"))
    {
        return Uart::Mode::Rs232;
    }
    else if (compareNoCase(str, "rs485"))
    {
        return Uart::Mode::Rs485;
    }
    else if (compareNoCase(str, "rs485t") || compareNoCase(str, "rs485terminated"))
    {
        return Uart::Mode::Rs485Terminated;
    }

    return Uart::Mode::Unknown;
}
//--------------------------------------------------------------------------------------------------
Uart::Parity StringUtils::toUartParity(const std::string& str)
{
    if (compareNoCase(str, "none") || compareNoCase(str, "off"))
    {
        return Uart::Parity::None;
    }
    if (compareNoCase(str, "odd"))
    {
        return Uart::Parity::Odd;
    }
    if (compareNoCase(str, "even"))
    {
        return Uart::Parity::Even;
    }
    if (compareNoCase(str, "mark"))
    {
        return Uart::Parity::Mark;
    }
    if (compareNoCase(str, "space"))
    {
        return Uart::Parity::Space;
    }
    return Uart::Parity::Unknown;
}
//--------------------------------------------------------------------------------------------------
Uart::StopBits StringUtils::toUartStopBits(const std::string& str)
{
    if ((str == "1") || compareNoCase(str, "one"))
    {
        return Uart::StopBits::One;
    }
    if ((str == "1.5") || compareNoCase(str, "OneAndHalf"))
    {
        return Uart::StopBits::OneAndHalf;
    }
    if ((str == "2") || compareNoCase(str, "two"))
    {
        return Uart::StopBits::Two;
    }

    return Uart::StopBits::Unknown;
}
//--------------------------------------------------------------------------------------------------
Device::PhyPortMode StringUtils::toPhyPortMode(const std::string& str)
{
    if (str == "phy_auto")
    {
        return Device::PhyPortMode::Auto;
    }
    if (str == "10_base_h")
    {
        return Device::PhyPortMode::Base10TxHalf;
    }
    if (str == "10_base_f")
    {
        return Device::PhyPortMode::Base10TxFull;
    }
    if (str == "100_base_h")
    {
        return Device::PhyPortMode::Base100TxHalf;
    }
    if (str == "100_base_f")
    {
        return Device::PhyPortMode::Base100TxFull;
    }

    return Device::PhyPortMode::Unknown;
}
//--------------------------------------------------------------------------------------------------
Device::PhyMdixMode StringUtils::toPhyMdixMode(const std::string& str)
{
    if (str == "mdix_normal")
    {
        return Device::PhyMdixMode::Normal;
    }
    if (str == "mdix_swap")
    {
        return Device::PhyMdixMode::Swapped;
    }
    if (str == "mdix_auto")
    {
        return Device::PhyMdixMode::Auto;
    }

    return Device::PhyMdixMode::Unknown;
}
//--------------------------------------------------------------------------------------------------
uint32_t StringUtils::toIp(const std::string& str, bool_t& error)
{
    uint_t index = 0;
    uint_t ip = 0;

    for (uint_t i = 0; i < 4; i++)
    {
        uint_t num = toUint(str, index, 0, error);
        if (error || num > 255)
        {
            return 0;
        }
        ip |= num << (8 * i);
        if ((index == str.length() || str[index] != '.') && (i != 3))
        {
            return 0;
        }
        index++;
    }

    return static_cast<uint32_t>(ip);
}
//--------------------------------------------------------------------------------------------------
bool_t StringUtils::toMacAddress(const std::string& str, uint8_t(&macAddress)[6])
{
    uint_t index = 0;
    bool_t error = true;

    for (uint_t i = 0; i < 6; i++)
    {
        uint_t num = hexToUint(str, index, 2, error);
        if (num > 255 || error)
        {
            error = true;
            break;
        }
        macAddress[i] = static_cast<uint8_t>(num);

        if (i < 5 && str[index] != ':')
        {
            error = true;
            break;
        }
        index++;
    }

    if (error)
    {
        for (uint_t i = 0; i < sizeof(macAddress); i++)
        {
            macAddress[i] = 0;
        }
    }

    return !error;
}
//--------------------------------------------------------------------------------------------------
uint_t StringUtils::copyMax(const std::string& str, uint8_t* buf, uint_t size)
{
    if (size && buf)
    {
        size--;
        size = Math::min<uint_t>(str.length(), size);
        Mem::memcpy(buf, str.c_str(), size);
        buf[size++] = 0;
    }
    return size;
}
//--------------------------------------------------------------------------------------------------
uint_t StringUtils::copyOrFill(const std::string& str, uint8_t* ptr, uint_t size)
{
    for (uint_t i = 0; i < size; i++)
    {
        *ptr++ = i < str.size() ? str[i] : 0;
    }
    return size;
}
//--------------------------------------------------------------------------------------------------
bool_t StringUtils::compareNoCase(const std::string& str1, const std::string& str2)
{
    if (str1.size() == str2.size())
    {
        for (size_t i = 0; i < str1.size(); ++i)
        {
            if (!((str1[i] == str2[i]) || (((str1[i] - str2[i]) == 32) && str2[i] >= 'A' && str2[i] <= 'Z') || (((str2[i] - str1[i]) == 32) && str1[i] >= 'A' && str1[i] <= 'Z')))
            {
                return false;
            }
        }

        return true;
    }
    return false;
}
//--------------------------------------------------------------------------------------------------
uint_t parseDigits(const char* str, uint_t& digits)
{
    uint_t num = 0;
    uint_t count = 0;
    bool_t overflow = false;
    int_t chars = digits;

    if (chars == 0)
    {
        chars = -1;
    }

    while (!overflow && chars && *str >= '0' && *str <= '9')
    {
        num = (num * 10) + (*str++ - '0');
        chars--;
        count++;
    }

    if (overflow || chars > 0)
    {
        num = 0;
        count = 0;
    }

    digits = count;
    return num;
}
//--------------------------------------------------------------------------------------------------
