//------------------------------------------ Includes ----------------------------------------------

#include "comms/protocols/cobs.h"
#include "platform/debug.h"

using namespace IslSdk;

//--------------------------------------------------------------------------------------------------
Cobs::Cobs(uint_t mtu) : Codec(mtu, Type::Cobs)
{
    m_blockSize = 0;
    m_blockCount = 0;
    m_size = 0;
    m_waitForSof = true;
}
//--------------------------------------------------------------------------------------------------
Cobs::~Cobs()
{
}
//--------------------------------------------------------------------------------------------------
uint_t Cobs::encode(const uint8_t* data, uint_t size, uint8_t* buf, uint_t bufSize)
{
    uint8_t* frame;
    uint8_t* idx;
    uint8_t blockLength;
    uint_t maxSize;

    maxSize = size + 3 + (size / 254);

    if (maxSize > bufSize)
    {
        debugLog("Cobs", "Cobs encoded frame (%u) would exceeds tx buffer size (%u), discarding data", FMT_U(maxSize), FMT_U(bufSize));
        return 0;
    }

    frame = buf;
    blockLength = 1;
    *frame++ = 0;
    idx = frame;
    frame++;

    while (size)
    {
        if (*data == 0)                 // Data is 0 so must be removed
        {
            *idx = blockLength;
            blockLength = 1;
            idx = frame;
            frame++;
        }
        else
        {
            *frame++ = *data;
            blockLength++;              // Count the size of this block
            if (blockLength == 0xff)    // Reached max block size
            {
                *idx = blockLength;
                blockLength = 1;
                idx = frame;
                frame++;
            }
        }
        data++;
        size--;
    }

    *idx = blockLength;
    *frame++ = 0;

    return (uint_t)(frame - buf);
}
//--------------------------------------------------------------------------------------------------
uint_t Cobs::decode(const uint8_t* data, uint_t* size)
{
    uint_t frameLength;

    while (*size)
    {
        (*size)--;
        if (*data == 0)                 // Start or end of frame marker
        {
            m_waitForSof = false;
            if (m_size && m_blockCount == 0)
            {
                frameLength = m_size;
                m_size = 0;
                m_blockCount = 0;
                m_blockSize = 0xff;
                return frameLength;
            }
            else if (m_size)
            {
                debugLog("Cobs", "Cobs rx frame incomplete, discarding data");
            }

            m_size = 0;
            m_blockCount = 0;
            m_blockSize = 0xff;
        }
        else if (!m_waitForSof)
        {
            if (m_blockCount)
            {
                m_frameBuf[m_size++] = *data;
            }
            else
            {
                if (m_blockSize != 0xff)
                {
                    m_frameBuf[m_size++] = 0;
                }
                m_blockSize = m_blockCount = *data;

                if ((m_size + m_blockSize) > m_frameBuf.size())
                {
                    m_size = 0;
                    m_waitForSof = true;
                    debugLog("Cobs", "Cobs rx frame exceeds buffer size, discarding data");
                }
            }
            m_blockCount--;
        }
        data++;
    }
    return 0;
}
//--------------------------------------------------------------------------------------------------
