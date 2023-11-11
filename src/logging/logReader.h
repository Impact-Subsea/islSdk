#ifndef LOGREADER_H_
#define LOGREADER_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"
#include "logging/logWriter.h"
#include "types/sigSlot.h"

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    class LogReader : public LogWriter
    {
    public:
        struct RecordData
        {
            uint8_t trackId;                        ///< Id of the track
            uint32_t timeMs;                        ///< Number of milliseconds since the time stamp { @link logInfo_t startTime } for this record
            uint_t recordIndex;                     ///< Index of this record in the track
            LogFile::RecordHeader::Type recordType; ///< Type of record
            uint8_t dataType;                       ///< Type of data in the record
            std::vector<uint8_t> data;

            RecordData(LogFile::RecordHeader hdr, uint_t recordIndex) :
                trackId(hdr.trackId),
                timeMs(hdr.timeMs),
                recordIndex(recordIndex),
                recordType(hdr.recordType),
                dataType(hdr.dataType),
                data(hdr.dataSize) {}
        };

        LogReader();
        virtual ~LogReader();
        virtual bool_t open(const std::string& filename);
        void play(real_t playSpeed);
        void reset();
        void seek(uint_t index);
        virtual bool_t close();
        void process();
        virtual void emitRecord(const RecordData& record);

        Signal<LogReader&, const RecordData&> onRecord;
        Signal<LogReader&, const std::string&> onError;

    private:
        void playlog(uint_t index);

        real_t m_playSpeed;
        real_t m_playTimer;
        int_t m_currentIndex;
        uint64_t m_lastTimeMs;
    };
}

//--------------------------------------------------------------------------------------------------
#endif
