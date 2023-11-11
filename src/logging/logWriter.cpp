//------------------------------------------ Includes ----------------------------------------------

#include "logging/logWriter.h"
#include "utils/stringUtils.h"
#include "platform/timeUtils.h"
#include "maths/maths.h"

using namespace IslSdk;

//--------------------------------------------------------------------------------------------------
LogWriter::LogWriter() : m_maxFileSize(1024 * 1024 * 1024), m_fileCount(0)
{
}
//--------------------------------------------------------------------------------------------------
LogWriter::~LogWriter()
{
}
//--------------------------------------------------------------------------------------------------
bool_t LogWriter::startNewFile(const std::string& filename)
{
    uint_t pos = filename.find_last_of('.');

    if (pos !=std::string::npos)
    {
        m_filename = filename.substr(0, pos);
    }
    else
    {
        m_filename = filename;
    }
    m_fileCount = 0;

    return m_file.startNew(m_filename + ".islog", Time::getTimeMs()) == LogFile::Error::None;
}
//--------------------------------------------------------------------------------------------------
uint8_t LogWriter::addTrack(uint8_t dataType, const uint8_t* data, uint_t size)
{
    uint8_t trackId = 0;

    if (m_file.tracks.size() < 255)
    {
        LogFile::Track* track = m_file.addTrack(static_cast<uint8_t>(m_file.tracks.size() + 1), dataType, data, size);
        if (track)
        {
            trackId = track->id;
        }
    }

    return trackId;
}
//--------------------------------------------------------------------------------------------------
bool_t LogWriter::addTrackData(uint8_t trackId, const void* data, uint_t size, uint8_t dataType, bool_t canSkip)
{
    bool_t ok = false;
    LogFile::Track* track = m_file.findTrack(trackId);

    if (track)
    {
        LogFile::RecordHeader recordHeader;
        recordHeader.timeMs = static_cast<uint32_t>(Time::getTimeMs() - m_file.timeMs);
        recordHeader.trackId = track->id;
        recordHeader.canSkip = canSkip;
        recordHeader.recordType = LogFile::RecordHeader::Type::Data;
        recordHeader.dataType = dataType;
        recordHeader.dataSize = static_cast<uint32_t>(size);

        ok = m_file.logRecord(*track, recordHeader, data);

        if (ok && m_file.fileWritePosition() >= m_maxFileSize)
        {
            m_fileCount++;
            ok = m_file.startNew(m_filename + "-" + StringUtils::uintToStr(m_fileCount) + ".islog", Time::getTimeMs()) == LogFile::Error::None;
        }
    }
    return ok;
}
//--------------------------------------------------------------------------------------------------
void LogWriter::setMaxFileSize(uint32_t maxSize)
{
    maxSize = Math::max<uint32_t>(maxSize, 1024);
    m_maxFileSize = maxSize;
}
//--------------------------------------------------------------------------------------------------
bool_t LogWriter::close()
{
    return m_file.close();
}
//--------------------------------------------------------------------------------------------------
