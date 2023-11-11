//------------------------------------------ Includes ----------------------------------------------

#include "comms/protocols/nmea.h"
#include "utils/stringUtils.h"
#include <string>

using namespace IslSdk;

//--------------------------------------------------------------------------------------------------
Nmea::Nmea(uint_t mtu) : Codec(mtu, Type::Nmea), m_size(0)
{
}
//--------------------------------------------------------------------------------------------------
Nmea::~Nmea()
{
}
//--------------------------------------------------------------------------------------------------
uint_t Nmea::encode(const uint8_t* data, uint_t size, uint8_t* buf, uint_t bufSize)
{
    return 0;
}
//--------------------------------------------------------------------------------------------------
uint_t Nmea::decode(const uint8_t* data, uint_t* size)
{
    while (*size)
    {
        (*size)--;
        uint8_t byte = *data++;

        if (byte == '$')
        {
            m_size = 0;
        }

        if (m_size < m_frameBuf.size() - 1)
        {
            m_frameBuf[m_size++] = byte;
        }

        if (byte == '\n' && m_size)
        {
            m_frameBuf[m_size] = 0;
            return m_size;
        }
    }
    return 0;
}
//--------------------------------------------------------------------------------------------------
bool_t Nmea::checkCrc(const uint8_t* data, uint_t size)
{
    bool_t error = false;
    uint8_t checkSum = 0;
    uint8_t crc = 1;

    if (size > 5 && *data++ == '$')
    {
        size--;
        while (*data && *data != '*' && size)
        {
            checkSum ^= *data++;
            size--;
        }
        data++;

        const std::string crcStr = StringUtils::toStr(data, size);
        uint_t index = 0;
        crc = static_cast<uint8_t>(StringUtils::hexToUint(crcStr, index, 2, error));
    }

    return checkSum == crc && !error;
}
//--------------------------------------------------------------------------------------------------
