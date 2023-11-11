//------------------------------------------ Includes ----------------------------------------------

#include "gpsDevice.h"
#include "utils/stringUtils.h"

using namespace IslSdk;

//---------------------------------------------------------------------------------------------------
GpsDevice::GpsDevice() : NmeaDevice(NmeaDevice::Type::Gps)
{
}
//---------------------------------------------------------------------------------------------------
GpsDevice::GpsDevice(const SysPort::SharedPtr& sysPort) : NmeaDevice(sysPort, NmeaDevice::Type::Gps)
{
}
//---------------------------------------------------------------------------------------------------
GpsDevice::~GpsDevice()
{
}
//--------------------------------------------------------------------------------------------------
LoggingDevice::Type GpsDevice::getTrackData(std::vector<uint8_t>& buf)
{
    buf.push_back(static_cast<uint8_t>(NmeaDevice::Type::Gps));

    return LoggingDevice::Type::Nmea;
}
//---------------------------------------------------------------------------------------------------
bool_t GpsDevice::newSentence(const std::string& str)
{
    bool_t shouldLog = false;

    SentenceType sentenceType = getSentenceType(str);

    if (sentenceType != SentenceType::Unsupported)
    {
        shouldLog = true;
    }

    onData(*this, str);

    return shouldLog;
}
//---------------------------------------------------------------------------------------------------
GpsDevice::SentenceType GpsDevice::getSentenceType(const std::string& str)
{
    if (!str.empty())
    {
        if (str.rfind("$GPGLL", 0) == 0)
        {
            return SentenceType::Gll;
        }
        else if (str.rfind("$GPGGA", 0) == 0)
        {
            return SentenceType::Gga;
        }
        else if (str.rfind("$GPGSV", 0) == 0)
        {
            return SentenceType::Gsv;
        }
        else if (str.rfind("$GPGSA", 0) == 0)
        {
            return SentenceType::Gsa;
        }
        else if (str.rfind("$GPVTG", 0) == 0)
        {
            return SentenceType::Vtg;
        }
        else if (str.rfind("$GPRMC", 0) == 0)
        {
            return SentenceType::Rmc;
        }
    }

    return SentenceType::Unsupported;
}
//---------------------------------------------------------------------------------------------------
bool_t GpsDevice::parseStringGPGLL(const std::string& str, Gpgll& gpgll)
{
    bool_t error = true;

    if (str.rfind("$GPGLL", 0) == 0)
    {
        uint_t idx = 6;
        error = false;

        if (str[idx++] == ',' && str[idx] != ',')
        {
            gpgll.latitudeDeg = static_cast<real_t>(StringUtils::toUint(str, idx, 2, error));
            gpgll.latitudeDeg += StringUtils::toReal(str, idx, error) / 60.0;
        }

        if (str[idx++] == ',' && str[idx] != ',')
        {
            if (str[idx] == 'S')
            {
                gpgll.latitudeDeg *= -1.0;
            }

            error |= (str[idx] != 'S' && str[idx] != 'N');
            idx++;
        }

        if (str[idx++] == ',' && str[idx] != ',')
        {
            gpgll.longitudeDeg = static_cast<real_t>(StringUtils::toUint(str, idx, 3, error));
            gpgll.longitudeDeg += StringUtils::toReal(str, idx, error) / 60.0;
        }

        if (str[idx++] == ',' && str[idx] != ',')
        {
            if (str[idx] == 'W')
            {
                gpgll.longitudeDeg *= -1.0;
            }

            error |= (str[idx] != 'E' && str[idx] != 'W');
            idx++;
        }

        if (str[idx++] == ',' && str[idx] != ',')
        {
            gpgll.hour = StringUtils::toUint(str, idx, 2, error);
            gpgll.minute = StringUtils::toUint(str, idx, 2, error);
            gpgll.second = StringUtils::toReal(str, idx, error);
        }

        if (str[idx++] == ',' && str[idx] != ',')
        {
            gpgll.valid = str[idx] == 'A';
            error |= (str[idx] != 'A' && str[idx] != 'V');
            idx++;
        }

        if (str[idx++] == ',' && str[idx] != ',')
        {
            gpgll.mode = str[idx];
            error |= (str[idx] != 'A' && str[idx] != 'D' && str[idx] != 'E' && str[idx] != 'M' && str[idx] != 'N');
            idx++;
        }
    }
    return !error;
}
//---------------------------------------------------------------------------------------------------
bool_t GpsDevice::parseStringGPGGA(const std::string& str, Gpgga& gpgga)
{
    bool_t error = true;

    if (str.rfind("$GPGGA", 0) == 0)
    {
        uint_t idx = 6;
        error = false;

        if (str[idx++] == ',' && str[idx] != ',')
        {
            gpgga.hour = StringUtils::toUint(str, idx, 2, error);
            gpgga.minute = StringUtils::toUint(str, idx, 2, error);
            gpgga.second = StringUtils::toReal(str, idx, error);
        }

        if (str[idx++] == ',' && str[idx] != ',')
        {
            gpgga.latitudeDeg = static_cast<real_t>(StringUtils::toUint(str, idx, 2, error));
            gpgga.latitudeDeg += (StringUtils::toReal(str, idx, error) / 60.0);
        }

        if (str[idx++] == ',' && str[idx] != ',')
        {
            if (str[idx] == 'S')
            {
                gpgga.latitudeDeg *= -1.0;
            }

            error |= (str[idx] != 'S' && str[idx] != 'N');
            idx++;
        }

        if (str[idx++] == ',' && str[idx] != ',')
        {
            gpgga.longitudeDeg = static_cast<real_t>(StringUtils::toUint(str, idx, 3, error));
            gpgga.longitudeDeg += (StringUtils::toReal(str, idx, error) / 60.0);
        }

        if (str[idx++] == ',' && str[idx] != ',')
        {
            if (str[idx] == 'W')
            {
                gpgga.longitudeDeg *= -1.0;
            }

            error |= (str[idx] != 'E' && str[idx] != 'W');
            idx++;
        }


        if (str[idx++] == ',' && str[idx] != ',')
        {
            gpgga.quality = StringUtils::toUint(str, idx, 0, error);
        }

        if (str[idx++] == ',' && str[idx] != ',')
        {
            gpgga.satellitesInUse = StringUtils::toUint(str, idx, 0, error);
        }

        if (str[idx++] == ',' && str[idx] != ',')
        {
            gpgga.hdop = StringUtils::toReal(str, idx, error);
        }

        if (str[idx++] == ',' && str[idx] != ',')
        {
            gpgga.altitude = StringUtils::toReal(str, idx, error);
        }

        if (str[idx++] == ',' && str[idx] != ',')
        {
            if (str[idx++] != 'M')
            {
                error = true;
            }
        }

        if (str[idx++] == ',' && str[idx] != ',')
        {
            gpgga.undulation = StringUtils::toReal(str, idx, error);
        }

        if (str[idx++] == ',' && str[idx] != ',')
        {
            if (str[idx++] != 'M')
            {
                error = true;
            }
        }

        if (str[idx++] == ',' && str[idx] != ',')
        {
            gpgga.ageSeconds = StringUtils::toUint(str, idx, 0, error);
        }

        if (str[idx++] == ',' && str[idx] != '*')
        {
            gpgga.stationId = StringUtils::toUint(str, idx, 0, error);
        }
    }

    return !error;
}
//---------------------------------------------------------------------------------------------------
bool_t GpsDevice::parseStringGPGSV(const std::string& str, Gpgsv& gpgsv)
{
    bool_t error = true;

    if (str.rfind("$GPGSV", 0) == 0)
    {
        uint_t idx = 6;
        error = false;

        if (str[idx++] == ',' && str[idx] != ',')
        {
            gpgsv.totalMessageCount = StringUtils::toUint(str, idx, 0, error);
        }

        if (str[idx++] == ',' && str[idx] != ',')
        {
            gpgsv.messageNumber = StringUtils::toUint(str, idx, 0, error);
        }

        if (str[idx++] == ',' && str[idx] != ',')
        {
            gpgsv.satellitesInView = StringUtils::toUint(str, idx, 0, error);
        }

        for (uint_t i = 0; i < 4; i++)
        {
            if (str[idx++] == ',' && str[idx] != ',')
            {
                gpgsv.satellite[i].prn = StringUtils::toUint(str, idx, 0, error);
            }

            if (str[idx++] == ',' && str[idx] != ',')
            {
                gpgsv.satellite[i].elevationDeg = StringUtils::toUint(str, idx, 0, error);
            }

            if (str[idx++] == ',' && str[idx] != ',')
            {
                gpgsv.satellite[i].azimuthDeg = StringUtils::toUint(str, idx, 0, error);
            }

            if (str[idx++] == ',' && str[idx] != ',' && str[idx] != '*')
            {
                gpgsv.satellite[i].snr = StringUtils::toUint(str, idx, 0, error);
            }

            if (str[idx] == '*')
            {
                break;
            }
        }
    }
    return !error;
}
//---------------------------------------------------------------------------------------------------
bool_t GpsDevice::parseStringGPGSA(const std::string& str, Gpgsa& gpgsa)
{
    bool_t error = true;

    if (str.rfind("$GPGSA", 0) == 0)
    {
        uint_t idx = 6;
        error = false;

        if (str[idx++] == ',' && str[idx] != ',')
        {
            gpgsa.mode = str[idx++];
        }

        if (str[idx++] == ',' && str[idx] != ',')
        {
            gpgsa.fixType = StringUtils::toUint(str, idx, 0, error);
        }

        for (uint_t i = 0; i < 12; i++)
        {
            if (str[idx++] == ',' && str[idx] != ',')
            {
                gpgsa.prn[i] = StringUtils::toUint(str, idx, 0, error);
            }
        }

        if (str[idx++] == ',' && str[idx] != ',')
        {
            gpgsa.pdop = StringUtils::toReal(str, idx, error);
        }

        if (str[idx++] == ',' && str[idx] != ',')
        {
            gpgsa.hdop = StringUtils::toReal(str, idx, error);
        }

        if (str[idx++] == ',' && str[idx] != '*')
        {
            gpgsa.vdop = StringUtils::toReal(str, idx, error);
        }
    }
    return !error;
}
//---------------------------------------------------------------------------------------------------
bool_t GpsDevice::parseStringGPVTG(const std::string& str, Gpvtg& gpvtg)
{
    bool_t error = true;

    if (str.rfind("$GPVTG", 0) == 0)
    {
        uint_t idx = 6;
        error = false;

        if (str[idx++] == ',' && str[idx] != ',')
        {
            gpvtg.headingTrue = StringUtils::toReal(str, idx, error);
        }

        if (str[idx++] == ',' && str[idx] != ',')
        {
            if (str[idx++] != 'T')
            {
                error = true;
            }
        }

        if (str[idx++] == ',' && str[idx] != ',')
        {
            gpvtg.headingMag = StringUtils::toReal(str, idx, error);
        }

        if (str[idx++] == ',' && str[idx] != ',')
        {
            if (str[idx++] != 'M')
            {
                error = true;
            }
        }

        if (str[idx++] == ',' && str[idx] != ',')
        {
            gpvtg.speedKn = StringUtils::toReal(str, idx, error);
        }

        if (str[idx++] == ',' && str[idx] != ',')
        {
            if (str[idx++] != 'N')
            {
                error = true;
            }
        }

        if (str[idx++] == ',' && str[idx] != ',')
        {
            gpvtg.speedKm = StringUtils::toReal(str, idx, error);
        }

        if (str[idx++] == ',' && str[idx] != ',')
        {
            if (str[idx++] != 'K')
            {
                error = true;
            }
        }

        if (str[idx++] == ',' && str[idx] != '*')
        {
            gpvtg.mode = str[idx];
            error |= (str[idx] != 'A' && str[idx] != 'D' && str[idx] != 'E' && str[idx] != 'M' && str[idx] != 'N');
        }
    }
    return !error;
}
//---------------------------------------------------------------------------------------------------
bool_t GpsDevice::parseStringGPRMC(const std::string& str, Gprmc& gprmc)
{
    bool_t error = true;

    if (str.rfind("$GPRMC", 0) == 0)
    {
        uint_t idx = 6;
        error = false;

        if (str[idx++] == ',' && str[idx] != ',')
        {
            gprmc.hour = StringUtils::toUint(str, idx, 2, error);
            gprmc.minute = StringUtils::toUint(str, idx, 2, error);
            gprmc.second = StringUtils::toReal(str, idx, error);
        }

        if (str[idx++] == ',' && str[idx] != ',')
        {
            gprmc.valid = str[idx] == 'A';
            error |= (str[idx] != 'A' && str[idx] != 'V');
            idx++;
        }

        if (str[idx++] == ',' && str[idx] != ',')
        {
            gprmc.latitudeDeg = static_cast<real_t>(StringUtils::toUint(str, idx, 2, error));
            gprmc.latitudeDeg += (StringUtils::toReal(str, idx, error) / 60.0);
        }

        if (str[idx++] == ',' && str[idx] != ',')
        {
            if (str[idx] == 'S')
            {
                gprmc.latitudeDeg *= -1.0;
            }

            error |= (str[idx] != 'S' && str[idx] != 'N');
            idx++;
        }

        if (str[idx++] == ',' && str[idx] != ',')
        {
            gprmc.longitudeDeg = static_cast<real_t>(StringUtils::toUint(str, idx, 3, error));
            gprmc.longitudeDeg += (StringUtils::toReal(str, idx, error) / 60.0);
        }

        if (str[idx++] == ',' && str[idx] != ',')
        {
            if (str[idx] == 'W')
            {
                gprmc.longitudeDeg *= -1.0;
            }

            error |= (str[idx] != 'E' && str[idx] != 'W');
            idx++;
        }

        if (str[idx++] == ',' && str[idx] != ',')
        {
            gprmc.speedKn = StringUtils::toReal(str, idx, error);
        }

        if (str[idx++] == ',' && str[idx] != ',')
        {
            gprmc.headingTrue = StringUtils::toReal(str, idx, error);
        }

        if (str[idx++] == ',' && str[idx] != ',')
        {
            gprmc.date = StringUtils::toUint(str, idx, 2, error);
            gprmc.month = StringUtils::toUint(str, idx, 2, error);
            gprmc.year = StringUtils::toUint(str, idx, 2, error);
        }

        if (str[idx++] == ',' && str[idx] != ',')
        {
            gprmc.magVar = StringUtils::toReal(str, idx, error);
        }

        if (str[idx++] == ',' && str[idx] != ',')
        {
            if (str[idx] == 'E')
            {
                gprmc.magVar *= -1.0;
            }

            error |= (str[idx] != 'E' && str[idx] != 'W');
            idx++;
        }

        if (str[idx++] == ',' && str[idx] != '*')
        {
            gprmc.mode = str[idx];
            error |= (str[idx] != 'A' && str[idx] != 'D' && str[idx] != 'E' && str[idx] != 'M' && str[idx] != 'N');
        }
    }
    return !error;
}
//---------------------------------------------------------------------------------------------------
