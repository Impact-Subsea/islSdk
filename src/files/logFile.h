#ifndef LOGFILE_H_
#define LOGFILE_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"
#include "platform/mem.h"
#include <fstream>
#include <vector>

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    class LogFile
    {
    public:
        struct FileHeader
        {
            uint32_t id;
            uint8_t version;
            uint64_t timeMs;
            uint32_t indexPosition;
        };

        struct RecordHeader
        {
            enum class Type { Meta = 1, Data, Index, Track };
            uint32_t timeMs;
            uint8_t trackId;
            bool_t canSkip;
            Type recordType;
            uint32_t dataSize;
            uint8_t dataType;
        };

        struct Track
        {
            uint8_t id;
            uint32_t fileOffset;
            uint32_t startIndex;
            uint32_t recordCount;
            uint32_t startTime;
            uint32_t durationMs;
            uint8_t dataType;
            std::vector<uint8_t> data;

            Track(uint8_t id, uint8_t dataType, const uint8_t* data, uint_t size) :
                id(id), fileOffset(0), dataType(dataType), startIndex(0), recordCount(0), startTime(0), durationMs(0), data(size)
            {
                if (data && size)
                {
                    Mem::memcpy(&this->data[0], data, size);
                }
            }
        };

        struct RecordIndex
        {
            uint8_t trackId;
            uint32_t fileOffset;
            uint32_t timeMs;
            bool_t canSkip;
            RecordHeader::Type type;
            RecordIndex(uint8_t trackId, uint32_t fileOffset, uint32_t timeMs, bool_t canSkip, RecordHeader::Type type) :
                trackId(trackId), fileOffset(fileOffset), timeMs(timeMs), canSkip(canSkip), type(type) {}
        };

        enum class Error { None, CannotOpen, CannotCreate, CannotWrite, Damaged };

        LogFile();
        ~LogFile();
        LogFile::Error open(const std::string& fileName);
        LogFile::Error repair(const std::string& fileName);
        LogFile::Error startNew(const std::string& fileName, uint64_t timeMs);
        Track* addTrack(uint8_t trackId, uint8_t dataType, const uint8_t* data, uint_t size);
        bool_t logRecord(Track& track, const RecordHeader& recordHeader, const void* data);
        bool_t readRecordHeader(RecordHeader& recordHeader, uint_t position);
        bool_t readRecordData(uint8_t* data, uint_t size);
        bool_t close();
        Track* findTrack(uint8_t id);

        const uint64_t& timeMs = m_timeMs;
        const uint32_t& durationMs = m_durationMs;
        const std::vector<RecordIndex>& records = m_records;
        const std::vector<Track>& tracks = m_tracks;
        uint_t fileWritePosition() const { return m_fileWritePosition; }

    private:
        std::fstream m_file;
        uint64_t m_timeMs;
        uint32_t m_durationMs;
        bool_t m_isModified;
        uint32_t m_fileWritePosition;
        std::vector<RecordIndex> m_records;
        std::vector<Track> m_tracks;

        void addIndex(Track& track, uint32_t position, uint32_t timeMs, RecordHeader::Type type, bool_t canSkip);
        bool_t writeRecord(Track& track, const RecordHeader& recordHeader, const void* data);
        bool_t readFileIndexes(uint_t indexPosition, uint_t version);
        bool_t writeFileIndexes();
        bool_t readFileHeader(FileHeader& fileHeader);
        bool_t writeFileHeader(const FileHeader& fileHeader);
    };
}

//--------------------------------------------------------------------------------------------------
#endif
