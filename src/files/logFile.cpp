//------------------------------------------ Includes ----------------------------------------------

#include "logFile.h"
#include "platform/file.h"
#include "platform/mem.h"

using namespace IslSdk;

//--------------------------------------------------------------------------------------------------

const uint32_t logFileHeaderId = 0xc0debe02;
const uint32_t fileHeaderSize = 17;
const uint32_t recordHeaderSize = 11;
const uint32_t indexHeaderSize = 11;
const uint32_t indexRecordSize = 10;

//--------------------------------------------------------------------------------------------------
LogFile::LogFile() : m_timeMs(0) , m_isModified(false), m_fileWritePosition(0), m_durationMs(0)
{
}
//--------------------------------------------------------------------------------------------------
LogFile::~LogFile()
{
    close();
}
//--------------------------------------------------------------------------------------------------
LogFile::Error LogFile::open(const std::string& fileName)
{
    LogFile::LogFile::Error err = LogFile::Error::CannotOpen;

    m_tracks.clear();
    m_records.clear();

    m_file.open(fileName, std::fstream::in | std::fstream::out | std::fstream::binary);

    if (!m_file.fail())
    {
        bool_t ok = false;
        LogFile::FileHeader fileHeader;
        if (readFileHeader(fileHeader))
        {
            if (fileHeader.id == logFileHeaderId && (fileHeader.version == 1 || fileHeader.version == 2) && fileHeader.indexPosition)
            {
                m_timeMs = fileHeader.timeMs;
                m_fileWritePosition = fileHeader.indexPosition;

                ok = readFileIndexes(fileHeader.indexPosition, fileHeader.version);
            }
        }

        if (!ok)
        {
            m_tracks.clear();
            m_records.clear();

            err = LogFile::Error::Damaged;

            if (m_file.is_open())
            {
                m_file.close();
            }
        }
        else
        {
            err = LogFile::Error::None;
        }
    }

    return err;
}
//--------------------------------------------------------------------------------------------------
LogFile::Error LogFile::repair(const std::string& fileName)
{
    LogFile::LogFile::Error err = LogFile::Error::Damaged;

    m_file.open(fileName, std::fstream::in | std::fstream::out | std::fstream::binary);

    if (!m_file.fail())
    {
        m_isModified = true;
        m_fileWritePosition = 0;
        m_records.clear();
        m_tracks.clear();

        LogFile::FileHeader fileHeader;
        if (readFileHeader(fileHeader))
        {
            if (fileHeader.id == logFileHeaderId && fileHeader.version == 2)
            {
                bool_t ok = true;
                uint32_t filePosition = fileHeaderSize;
                m_timeMs = fileHeader.timeMs;

                LogFile::RecordHeader recordHeader;
                while (ok && !m_file.eof())
                {
                    if (readRecordHeader(recordHeader, filePosition))
                    {
                        switch (recordHeader.recordType)
                        {
                        case RecordHeader::Type::Track:
                        {
                            std::vector<uint8_t> trackData(recordHeader.dataSize);
                            ok = readRecordData(&trackData[0], trackData.size());

                            Track* track = findTrack(recordHeader.trackId);
                            if (!track)
                            {
                                track = addTrack(recordHeader.trackId, recordHeader.dataType, &trackData[0], recordHeader.dataSize);
                                track->fileOffset = filePosition;
                            }
                            break;
                        }
                        case RecordHeader::Type::Data:
                        {
                            Track* track = findTrack(recordHeader.trackId);
                            if (track)
                            {
                                addIndex(*track, filePosition, recordHeader.timeMs, recordHeader.recordType, recordHeader.canSkip);
                            }
                            break;
                        }
                        default:
                            break;
                        }

                        filePosition += recordHeader.dataSize + recordHeaderSize;
                    }
                }

                if (ok)
                {
                    m_fileWritePosition = filePosition;
                    if (writeFileIndexes())
                    {
                        err = LogFile::Error::None;
                    }
                }

                m_records.clear();
                m_tracks.clear();
            }
        }

        if (m_file.is_open())
        {
            m_file.close();
        }
    }
    else
    {
        err = LogFile::Error::CannotOpen;
    }

    return err;
}
//--------------------------------------------------------------------------------------------------
LogFile::Error LogFile::startNew(const std::string& fileName, uint64_t timeMs)
{
    LogFile::Error err = LogFile::Error::None;

    if (m_file.is_open())
    {
        close();
    }

    for (Track track : m_tracks)
    {
        track.startIndex = 0;
        track.recordCount = 0;
        track.startTime = 0;
        track.durationMs = 0;
    }

    m_records.clear();
    m_timeMs = timeMs;
    m_fileWritePosition = fileHeaderSize;
    m_isModified = false;

    LogFile::FileHeader fileHeader;
    fileHeader.id = logFileHeaderId;
    fileHeader.version = 2;
    fileHeader.timeMs = timeMs;
    fileHeader.indexPosition = 0;

    File::createDir(fileName);
    m_file.open(fileName, std::fstream::out | std::fstream::binary | std::fstream::trunc);

    if (!m_file.fail())
    {
        m_isModified = true;
        if (!writeFileHeader(fileHeader))
        {
            m_file.close();
            err = LogFile::Error::CannotWrite;
        }
    }
    else
    {
        err = LogFile::Error::CannotOpen;
    }

    return err;
}
//--------------------------------------------------------------------------------------------------
LogFile::Track* LogFile::addTrack(uint8_t trackId, uint8_t dataType, const uint8_t* data, uint_t size)
{
    for (const Track& track : m_tracks)
    {
        if (track.id == trackId)
        {
            return nullptr;
        }
    }

    m_tracks.emplace_back(trackId, dataType, data, size);

    return &m_tracks.back();
}
//--------------------------------------------------------------------------------------------------
bool_t LogFile::logRecord(Track& track, const RecordHeader& recordHeader, const void* data)
{
    bool_t ok = true;

    if (track.recordCount == 0)
    {
        LogFile::RecordHeader rh;
        rh.dataSize = static_cast<uint32_t>(track.data.size());
        rh.timeMs = recordHeader.timeMs;
        rh.trackId = track.id;
        rh.canSkip = true;
        rh.recordType = RecordHeader::Type::Track;
        rh.dataType = track.dataType;

        track.fileOffset = m_fileWritePosition;
        ok = writeRecord(track, rh, &track.data[0]);
    }

    if (ok)
    {
        m_isModified = true;
        ok = writeRecord(track, recordHeader, data);
    }

    return ok;
}
//--------------------------------------------------------------------------------------------------
bool_t LogFile::readRecordHeader(RecordHeader& recordHeader, uint_t position)
{
    uint8_t buf[recordHeaderSize];

    if (!m_file.seekg(position))
    {
        return false;
    }

    m_file.read(reinterpret_cast<char*>(&buf[0]), recordHeaderSize);
    if (m_file.gcount() != recordHeaderSize)
    {
        return false;
    }

    const uint8_t* ptr = &buf[0];
    recordHeader.dataSize = Mem::get32Bit(&ptr) - recordHeaderSize;
    recordHeader.timeMs = Mem::get32Bit(&ptr);
    recordHeader.trackId = *ptr++;
    recordHeader.canSkip = (*ptr & 0x80) != 0;
    recordHeader.recordType = static_cast<RecordHeader::Type>(*ptr++ & 0x7f);
    recordHeader.dataType = *ptr;

    return true;
}
//--------------------------------------------------------------------------------------------------
bool_t LogFile::readRecordData(uint8_t* data, uint_t size)
{
    if (size != 0)
    {
        m_file.read(reinterpret_cast<char*>(data), size);
        return m_file.gcount() == static_cast<int_t>(size);
    }

    return true;
}
//--------------------------------------------------------------------------------------------------
bool_t LogFile::close()
{
    bool_t ok = false;

    if (m_file.is_open())
    {
        ok = writeFileIndexes();
        m_file.close();
    }

    return ok;
}
//--------------------------------------------------------------------------------------------------
LogFile::Track* LogFile::findTrack(uint8_t id)
{
    for (Track& track : m_tracks)
    {
        if (track.id == id)
        {
            return &track;
        }
    }
    return nullptr;
}
//--------------------------------------------------------------------------------------------------
void LogFile::addIndex(Track& track, uint32_t position, uint32_t timeMs, RecordHeader::Type type, bool_t canSkip)
{
    if (track.recordCount == 0)
    {
        track.startTime = timeMs;
        track.startIndex = static_cast<uint32_t>(m_records.size());
    }

    m_records.emplace_back(track.id, position, timeMs, canSkip, type);

    track.recordCount++;
    track.durationMs = timeMs - track.startTime;

    if ((track.startTime + timeMs) > m_durationMs)
    {
        m_durationMs = track.startTime + timeMs;
    }
}
//--------------------------------------------------------------------------------------------------
bool_t LogFile::writeRecord(Track& track, const RecordHeader& recordHeader, const void* data)
{
    if (!m_file.seekp(m_fileWritePosition))
    {
        return false;
    }

    if (recordHeader.recordType == RecordHeader::Type::Data)
    {
        addIndex(track, m_fileWritePosition, recordHeader.timeMs, recordHeader.recordType, recordHeader.canSkip);
    }

    uint8_t buf[recordHeaderSize];
    uint8_t* ptr = &buf[0];
    Mem::pack32Bit(&ptr, recordHeaderSize + recordHeader.dataSize);
    Mem::pack32Bit(&ptr, recordHeader.timeMs);
    *ptr++ = recordHeader.trackId;
    *ptr++ = static_cast<uint8_t>(recordHeader.recordType) | static_cast<uint8_t>(recordHeader.canSkip << 7);
    *ptr++ = recordHeader.dataType;

    m_file.write(reinterpret_cast<const char*>(&buf[0]), recordHeaderSize);
    if (m_file.fail())
    {
        return false;
    }

    m_fileWritePosition += recordHeaderSize;

    if (recordHeader.dataSize)
    {
        m_file.write(static_cast<const char*>(data), recordHeader.dataSize);
        if (m_file.fail())
        {
            return false;
        }
        m_fileWritePosition += recordHeader.dataSize;
    }

    return true;
}
//--------------------------------------------------------------------------------------------------
bool_t LogFile::readFileIndexes(uint_t indexPosition, uint_t version)
{
    LogFile::RecordHeader recordHeader;
    if (!readRecordHeader(recordHeader, indexPosition))
    {
        return false;
    }

    uint8_t buf[indexRecordSize * 50];
    m_file.read(reinterpret_cast<char*>(&buf[0]), 4);
    if (m_file.gcount() != 4)
    {
        return false;
    }

    uint32_t recordCount = Mem::get32Bit(&buf[0]);
    uint_t trackCount = recordHeader.dataType;

    if (version > 1)
    {
        if (recordHeader.recordType != RecordHeader::Type::Index)
        {
            return false;
        }

        for (uint_t i = 0; i < trackCount; i++)
        {
            m_file.read(reinterpret_cast<char*>(&buf[0]), 4);
            std::streampos readPos = m_file.tellg();

            if (m_file.gcount() != 4)
            {
                return false;
            }
            uint32_t trackPosition = Mem::get32Bit(&buf[0]);

            if (!readRecordHeader(recordHeader, trackPosition))
            {
                return false;
            }
            if (recordHeader.recordType != RecordHeader::Type::Track)
            {
                return false;
            }
            std::vector<uint8_t> trackData(recordHeader.dataSize);
            if (!readRecordData(&trackData[0], trackData.size()))
            {
                return false;
            }
            Track* track = addTrack(recordHeader.trackId, recordHeader.dataType, &trackData[0], recordHeader.dataSize);
            if (track == nullptr)
            {
                return false;
            }
            track->fileOffset = trackPosition;

            if (!m_file.seekg(readPos, std::ios::beg))
            {
                return false;
            }
        }
    }
    else
    {
        for (uint_t i = 0; i < trackCount; i++)
        {
            m_file.read(reinterpret_cast<char*>(&buf[0]), indexHeaderSize);

            if (m_file.gcount() != indexHeaderSize)
            {
                return false;
            }

            if (addTrack(buf[0], 0, &buf[0], indexHeaderSize) == nullptr)
            {
                return false;
            }
        }
    }

    m_file.read(reinterpret_cast<char*>(&buf[0]), sizeof(buf));
    uint_t bytesRead = m_file.gcount();
    while (bytesRead && recordCount)
    {
        const uint8_t* ptr = &buf[0];
        while (bytesRead >= indexRecordSize && recordCount)
        {
            uint32_t fileOffset = Mem::get32Bit(&ptr);
            uint32_t timeMs = Mem::get32Bit(&ptr);
            RecordHeader::Type type = static_cast<RecordHeader::Type>(*ptr & 0x7f);
            bool_t canSkip = (*ptr++ & 0x80) != 0;
            LogFile::Track* logTrack = findTrack(*ptr++);

            if (logTrack == nullptr)
            {
                return false;
            }

            addIndex(*logTrack, fileOffset, timeMs, type, canSkip);
            bytesRead -= indexRecordSize;
            recordCount--;
        }

        m_file.read(reinterpret_cast<char*>(&buf[0]), sizeof(buf));
        bytesRead = m_file.gcount();
    }

    m_file.clear();

    return true;
}
//--------------------------------------------------------------------------------------------------
bool_t LogFile::writeFileIndexes()
{
    if (!m_isModified)
    {
        return true;
    }

    m_file.clear();
    if (!m_file.seekp(m_fileWritePosition))
    {
        return false;
    }

    uint32_t recordCount = static_cast<uint32_t>(m_records.size());
    uint8_t buf[recordHeaderSize + 4];
    uint8_t* ptr = &buf[0];

    Mem::pack32Bit(&ptr, static_cast<uint32_t>(sizeof(buf) + (m_tracks.size() * 4) + (recordCount * indexRecordSize)));
    Mem::pack32Bit(&ptr, 0);
    *ptr++ = 0;
    *ptr++ = static_cast<uint8_t>(RecordHeader::Type::Index);
    *ptr++ = static_cast<uint8_t>(m_tracks.size());
    Mem::pack32Bit(&ptr, recordCount);

    m_file.write(reinterpret_cast<const char*>(&buf[0]), sizeof(buf));
    if (m_file.fail())
    {
        return false;
    }

    for (const Track& track : m_tracks)
    {
        Mem::pack32Bit(&buf[0], track.fileOffset);
        m_file.write(reinterpret_cast<const char*>(&buf[0]), 4);
        if (m_file.fail())
        {
            return false;
        }
    }

    for (const RecordIndex& record : m_records)
    {
        ptr = &buf[0];
        Mem::pack32Bit(&ptr, record.fileOffset);
        Mem::pack32Bit(&ptr, record.timeMs);
        *ptr++ = static_cast<uint8_t>(record.type) | static_cast<uint8_t>(record.canSkip << 7);
        *ptr = record.trackId;

        m_file.write(reinterpret_cast<const char*>(&buf[0]), indexRecordSize);
        if (m_file.fail())
        {
            return false;
        }
    }

    if (!m_file.seekp(fileHeaderSize - sizeof(uint32_t)))
    {
        return false;
    }
    m_file.clear();

    Mem::pack32Bit(&buf[0], m_fileWritePosition);
    m_file.write(reinterpret_cast<const char*>(&buf[0]), 4);

    return !m_file.fail();
}
//--------------------------------------------------------------------------------------------------
bool_t LogFile::readFileHeader(FileHeader& fileHeader)
{
    if (!m_file.seekg(0))
    {
        return false;
    }

    uint8_t buf[fileHeaderSize];
    m_file.read(reinterpret_cast<char*>(&buf[0]), fileHeaderSize);
    if (m_file.gcount() == fileHeaderSize)
    {
        const uint8_t* ptr = &buf[0];
        fileHeader.id = Mem::get32Bit(&ptr);
        fileHeader.version = *ptr++;
        fileHeader.timeMs = Mem::get64Bit(&ptr);
        fileHeader.indexPosition = Mem::get32Bit(&ptr);
        return true;
    }
    return false;
}
//--------------------------------------------------------------------------------------------------
bool_t LogFile::writeFileHeader(const FileHeader& fileHeader)
{
    if (!m_file.seekp(0))
    {
        return false;
    }

    uint8_t buf[fileHeaderSize];
    uint8_t* ptr = &buf[0];
    Mem::pack32Bit(&ptr, fileHeader.id);
    *ptr++ = fileHeader.version;
    Mem::pack64Bit(&ptr, fileHeader.timeMs);
    Mem::pack32Bit(&ptr, fileHeader.indexPosition);

    m_file.write(reinterpret_cast<const char*>(&buf[0]), sizeof(buf));
    return !m_file.fail();
}
//--------------------------------------------------------------------------------------------------
