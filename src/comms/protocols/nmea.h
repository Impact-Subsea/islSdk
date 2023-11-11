#ifndef NMEA_H_
#define NMEA_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"
#include "comms/protocols/codec.h"

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    class Nmea : public Codec
    {
    public:
        Nmea(uint_t mtu);
        ~Nmea();
        uint_t encode(const uint8_t* data, uint_t size, uint8_t* buf, uint_t bufSize) override;
        uint_t decode(const uint8_t* data, uint_t* size) override;
        static bool_t checkCrc(const uint8_t* data, uint_t size);

    private:
        uint_t m_size;
    };
}
//--------------------------------------------------------------------------------------------------
#endif
