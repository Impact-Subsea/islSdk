//------------------------------------------ Includes ----------------------------------------------

#include "logReader.h"
#include "platform/timeUtils.h"

using namespace IslSdk;

//--------------------------------------------------------------------------------------------------
LogReader::LogReader() : m_playSpeed(0), m_playTimer(0), m_currentIndex(-1), m_lastTimeMs(Time::getTimeMs())
{
}
//--------------------------------------------------------------------------------------------------
LogReader::~LogReader()
{
}
//--------------------------------------------------------------------------------------------------
bool_t LogReader::open(const std::string& filename)
{
    m_playSpeed = 0;
    m_playTimer = 0;
    m_currentIndex = -1;
    m_lastTimeMs = Time::getTimeMs();

    LogFile::Error error = m_file.open(filename);
    if (error != LogFile::Error::None)
    {
        if (error == LogFile::Error::Damaged)
        {
            onError(*this, "log file (" + filename + ") error, attempting to repair file");
            m_file.repair(filename);
            error = m_file.open(filename);
        }

        if (error != LogFile::Error::None)
        {
            if (error == LogFile::Error::CannotOpen)
            {
                onError(*this, "log file (" + filename + ") failed to open");
            }
            else if (error == LogFile::Error::Damaged)
            {
                onError(*this, "log file (" + filename + ") damaged beyond repair");
            }
        }
    }

    return error == LogFile::Error::None;
}
//--------------------------------------------------------------------------------------------------
void LogReader::play(real_t playSpeed)
{
    m_playSpeed = playSpeed;
}
//--------------------------------------------------------------------------------------------------
void LogReader::reset()
{
    m_playTimer = 0;
    m_currentIndex = -1;
}
//--------------------------------------------------------------------------------------------------
void LogReader::seek(uint_t index)
{
    if (m_file.records.size())
    {
        if (index >= m_file.records.size())
        {
            index = m_file.records.size() - 1;
        }

        m_playTimer = static_cast<real_t>(m_file.records[index].timeMs);
        playlog(index);
    }
}
//--------------------------------------------------------------------------------------------------
bool_t LogReader::close()
{
    return m_file.close();
}
//--------------------------------------------------------------------------------------------------
void LogReader::process()
{
    uint64_t timeMs = Time::getTimeMs();

    if (m_playSpeed != 0.0 && m_file.records.size())
    {
        real_t deltaMs = static_cast<real_t>(timeMs - m_lastTimeMs);
        m_playTimer += deltaMs * m_playSpeed;
        uint_t index = m_currentIndex < 0 ? 0 : m_currentIndex;
        real_t logTime = static_cast<real_t>(m_file.records[index].timeMs);

        if (m_playSpeed > 0.0)
        {
            uint_t maxIdx = m_file.records.size() - 1;
            while ((logTime <= m_playTimer) && (index < maxIdx))
            {
                index++;
                logTime = static_cast<real_t>(m_file.records[index].timeMs);
            }
        }
        else
        {
            while ((logTime >= m_playTimer) && index)
            {
                index--;
                logTime = static_cast<real_t>(m_file.records[index].timeMs);
            }
        }
        playlog(index);
    }

    m_lastTimeMs = timeMs;
}
//--------------------------------------------------------------------------------------------------
void LogReader::playlog(uint_t index)
{
    int_t recordsToSend = static_cast<int_t>(index) - m_currentIndex;
    int_t dir = recordsToSend < 0 ? -1 : 1;

    if (recordsToSend < 0)
    {
        recordsToSend = -recordsToSend;
        dir = -1;
    }

    while (recordsToSend)
    {
        m_currentIndex += dir;
        const LogFile::RecordIndex& recordIdx = m_file.records[m_currentIndex];

        if (recordsToSend < 50 || !recordIdx.canSkip)
        {
            if (recordIdx.type <= LogFile::RecordHeader::Type::Data)
            {
                LogFile::RecordHeader header;
                if (m_file.readRecordHeader(header, recordIdx.fileOffset))
                {
                    RecordData record(header, m_currentIndex);
                    if (m_file.readRecordData(&record.data[0], record.data.size()))
                    {
                        emitRecord(record);
                    }
                }
            }
        }
        recordsToSend--;
    }
}
//--------------------------------------------------------------------------------------------------
void LogReader::emitRecord(const RecordData& record)
{
    onRecord(*this, record);
}
//--------------------------------------------------------------------------------------------------
