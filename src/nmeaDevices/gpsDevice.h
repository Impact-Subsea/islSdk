#ifndef GPSDEVICE_H_
#define GPSDEVICE_H_

//------------------------------------------ Includes ----------------------------------------------

#include "nmeaDevice.h"

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    class GpsDevice : public NmeaDevice
    {
    public:
        GpsDevice();
        GpsDevice(const SysPort::SharedPtr& sysPort);
        ~GpsDevice();

        Signal<GpsDevice&, const std::string&> onData;

        /// Supported GPS string types
        enum class SentenceType { Unsupported, Gll, Gga, Gsv, Gsa, Vtg, Rmc };

        /// Struct populated from GPGLL string
        struct Gpgll
        {
            real_t latitudeDeg;         ///< Latitude in degrees
            real_t longitudeDeg;        ///< Longitude in degrees
            uint_t hour;                ///< UTC hours
            uint_t minute;              ///< UTC minutes
            real_t second;              ///< UTC seconds
            bool_t valid;               ///< Data valid if true
            uint8_t mode;               ///< 'A' = Autonomous, 'D' = Differential, 'E' = Estimated (dead reckoning), 'M' = Manual input, 'N' = Data not valid
            Gpgll() : latitudeDeg(0.0), longitudeDeg(0.0), hour(0), minute(0), second(0.0), valid(false), mode(0) {}
        };

        /// Struct populated from GPGGA string
        struct Gpgga
        {
            uint_t hour;                ///< UTC hours
            uint_t minute;              ///< UTC minutes
            real_t second;              ///< UTC seconds
            real_t latitudeDeg;         ///< Latitude in degrees
            real_t longitudeDeg;        ///< Longitude in degrees
            uint_t quality;             ///< 0 = no fix, 1-9 = fix
            uint_t satellitesInUse;     ///< Number of satellites in use
            real_t hdop;                ///< Horizontal dilution of precision
            real_t altitude;            ///< Antenna altitude above/below mean sea level in meters
            real_t undulation;          ///< The relationship between the geoid and the WGS84 ellipsoid in meters
            uint_t ageSeconds;          ///< Age of correction data
            uint_t stationId;           ///< Differential base station ID
            Gpgga() : hour(0), minute(0), second(0.0), latitudeDeg(0.0), longitudeDeg(0.0), quality(0), satellitesInUse(0), hdop(0.0), altitude(0.0), undulation(0.0), ageSeconds(0), stationId(0) {}
        };

        struct GpgsvSatellite
        {
            uint_t prn;                 ///< Satellite PRN number
            int_t elevationDeg;         ///< Elevation, degrees, 90 maximum
            uint_t azimuthDeg;          ///< Azimuth, degrees True, 000 to 359
            uint_t snr;                 ///< SNR 00-99 dB
            GpgsvSatellite() : prn(0), elevationDeg(0), azimuthDeg(0), snr(0) {}
        };

        /// Struct populated from GPGSV string
        struct Gpgsv
        {
            uint_t totalMessageCount;   ///< Total number of messages
            uint_t messageNumber;       ///< Message number of this message
            uint_t satellitesInView;    ///< Total number of satellites in view
            GpgsvSatellite satellite[4];
            Gpgsv() : totalMessageCount(0), messageNumber(0), satellitesInView(0) {}
        };

        /// Struct populated from GPGSA string
        struct Gpgsa
        {
            uint8_t mode;               ///< A = automatic, M = manual
            uint_t fixType;             ///< 1 = Fix not available, 2 = 2D, 3 = 3D
            uint_t prn[12];             ///< PRN number of the satellite, GPS = 1 to 32, SBAS = 33 to 64 (add 87 for PRN number), GLO = 65 to 96
            real_t pdop;                ///< Position dilution of precision
            real_t hdop;                ///< Horizontal dilution of precision
            real_t vdop;                ///< Vertical dilution of precision
            Gpgsa() : mode(0), fixType(0), prn{}, pdop(0.0), hdop(0.0), vdop(0.0) {}
        };

        /// Struct populated from GPVTG string
        struct Gpvtg
        {
            real_t headingTrue;         ///< Heading in degrees from true North
            real_t headingMag;          ///< Heading in degrees from magnetic North
            real_t speedKn;             ///< Speed in Knots
            real_t speedKm;             ///< Speed in km/hr
            uint8_t mode;               ///< 'A' = Autonomous, 'D' = Differential, 'E' = Estimated (dead reckoning), 'M' = Manual input, 'N' = Data not valid
            Gpvtg() : headingTrue(0.0), headingMag(0.0), speedKn(0.0), speedKm(0.0), mode(0) {}
        };

        /// Struct populated from GPRMC string
        struct Gprmc
        {
            uint_t hour;                ///< UTC hours
            uint_t minute;              ///< UTC minutes
            real_t second;              ///< UTC seconds
            bool_t valid;               ///< Data valid if true
            real_t latitudeDeg;         ///< Latitude in degrees
            real_t longitudeDeg;        ///< Longitude in degrees
            real_t speedKn;             ///< Speed in Knots
            real_t headingTrue;         ///< Heading in degrees from true North
            uint_t date;                ///< Date
            uint_t month;               ///< Month
            uint_t year;                ///< Year
            real_t magVar;              ///< Magnetic variation in degrees. Add to headingTrue to get mag heading. Negative values are East
            uint8_t mode;               ///< 'A' = Autonomous, 'D' = Differential, 'E' = Estimated (dead reckoning), 'M' = Manual input, 'N' = Data not valid
            Gprmc() : hour(0), minute(0), second(0.0), valid(false), latitudeDeg(0.0), longitudeDeg(0.0), speedKn(0.0), headingTrue(0.0), date(0), month(0), year(0), magVar(0.0), mode(0) {}
        };

        static SentenceType getSentenceType(const std::string& str);
        bool_t parseStringGPGLL(const std::string& str, Gpgll& gpgll);
        bool_t parseStringGPGGA(const std::string& str, Gpgga& gpgga);
        bool_t parseStringGPGSV(const std::string& str, Gpgsv& gpgsv);
        bool_t parseStringGPGSA(const std::string& str, Gpgsa& gpgsa);
        bool_t parseStringGPVTG(const std::string& str, Gpvtg& gpvtg);
        bool_t parseStringGPRMC(const std::string& str, Gprmc& gprmc);

    private:
        LoggingDevice::Type getTrackData(std::vector<uint8_t>& buf) override;
        bool_t newSentence(const std::string& str) override;
    };
}

//--------------------------------------------------------------------------------------------------

#endif
