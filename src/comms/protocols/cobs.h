#ifndef COBS_H_
#define COBS_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"
#include "comms/protocols/codec.h"

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    class Cobs : public Codec
    {
    public:
        Cobs(uint_t mtu);
        ~Cobs();
        uint_t encode(const uint8_t* data, uint_t size, uint8_t* buf, uint_t bufSize) override;
        uint_t decode(const uint8_t* data, uint_t* size) override;

    private:
        uint_t m_blockSize;
        uint_t m_blockCount;
        uint_t m_size;
        bool_t m_waitForSof;
    };
}
//--------------------------------------------------------------------------------------------------
#endif
