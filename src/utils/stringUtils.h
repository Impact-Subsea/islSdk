#ifndef STRINGUTILS_H_
#define STRINGUTILS_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"
#include "devices/device.h"
#include <string>

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    namespace StringUtils
    {
        std::string toStr(const uint8_t* str, uint_t maxSize);
        std::string uintToStr(uint_t number, uint_t leadingZeros = 0);
        std::string intToStr(int_t number, uint_t leadingZeros = 0, bool_t forceSign = false);
        std::string realToStr(real_t number, uint_t leadingZeros = 0, uint_t precision = 2, uint8_t format = 0, bool_t forceSign = false);
        std::string uintToHexStr(uint_t value, uint_t leadingZeros = 0, bool_t useCaps = false);

        std::string pidToStr(Device::Pid pid);
        std::string pnSnToStr(uint16_t pn, uint16_t sn);
        std::string bcdVersionToStr(uint16_t bcdVersion);
        std::string uartModeToStr(Device::UartMode mode);
        std::string uartParityToStr(Device::Parity parity);
        std::string uartStopBitsToStr(Device::StopBits stopBits);
        std::string phyPortModeToStr(Device::PhyPortMode portMode);
        std::string phyMdixModeToStr(Device::PhyMdixMode mode);
        std::string macAddressToStr(const uint8_t(&macAddress)[6]);
        std::string ipToStr(uint32_t ip);

        uint_t toUint(const std::string& str, bool_t& error);
        uint_t toUint(const std::string& str, uint_t& index, uint_t digits, bool_t& error);
        int_t toInt(const std::string& str, bool_t& error);
        int_t toInt(const std::string& str, uint_t& index, uint_t digits, bool_t& error);
        real_t toReal(const std::string& str, bool_t& error);
        real_t toReal(const std::string& str, uint_t& index, bool_t& error);
        uint_t hexToUint(const std::string& str, bool_t& error);
        uint_t hexToUint(const std::string& str, uint_t& index, uint_t digits, bool_t& error);

        Device::Pid toPid(const std::string& str);
        uint32_t toPnSn(const std::string& str, bool_t& error);
        uint16_t toBcdVersion(const std::string& str, bool_t& error);
        Device::UartMode toUartMode(const std::string& str);
        Device::Parity toUartParity(const std::string& str);
        Device::StopBits toUartStopBits(const std::string& str);
        Device::PhyPortMode toPhyPortMode(const std::string& str);
        Device::PhyMdixMode toPhyMdixMode(const std::string& str);
        bool_t toMacAddress(const std::string& str, uint8_t(&macAddress)[6]);
        uint32_t toIp(const std::string& str, bool_t& error);

        uint_t copyMax(const std::string& str, uint8_t* buf, uint_t size);
        uint_t copyOrFill(const std::string& str, uint8_t* ptr, uint_t size);
        bool_t compareNoCase(const std::string& str1, const std::string& str2);
    }
}
//--------------------------------------------------------------------------------------------------
#endif
