//------------------------------------------ Includes ----------------------------------------------

#include "logging/loggingDevice.h"
#include "platform/mem.h"

using namespace IslSdk;

//--------------------------------------------------------------------------------------------------
bool_t LoggingDevice::log(const uint8_t* data, uint_t size, uint8_t dataType, bool_t canSkip)
{
    if (m_active && m_logger)
    {
        return m_logger->addTrackData(m_trackId, data, size, dataType, canSkip);
    }
    return false;
}
//--------------------------------------------------------------------------------------------------
void LoggingDevice::setLogger(std::shared_ptr<LogWriter>& logger, const uint8_t* trackData, uint_t size)
{
    if (logger != nullptr)
    {
        if (m_logger.get() != logger.get())
        {
            m_logger = logger;
            std::vector<uint8_t> buf;
            Type deviceType = getTrackData(buf);
            buf.insert(buf.end(), trackData, trackData + size);

            if (buf.size())
            {
                m_trackId = logger->addTrack(static_cast<uint8_t>(deviceType), &buf[0], buf.size());
            }
            else
            {
                m_trackId = logger->addTrack(static_cast<uint8_t>(deviceType), nullptr, 0);
            }
        }
    }
    else
    {
        m_logger = nullptr;
        m_trackId = 0;
    }

    if (m_active)
    {
        stopLogging();
    }
}
//--------------------------------------------------------------------------------------------------
bool_t LoggingDevice::startLogging()
{
    m_active = m_logger != nullptr;
    return m_active;
}
//--------------------------------------------------------------------------------------------------
void LoggingDevice::stopLogging()
{
    m_active = false;
}
//--------------------------------------------------------------------------------------------------
