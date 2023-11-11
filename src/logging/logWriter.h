#ifndef LOGWRITER_H_
#define LOGWRITER_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"
#include "files/logFile.h"
#include "types/sigSlot.h"

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    class LogWriter
    {
    public:

        Signal<LogWriter&> onMaxFileSize;

        LogWriter();
        virtual ~LogWriter();
        bool_t startNewFile(const std::string& filename);
        uint8_t addTrack(uint8_t dataType, const uint8_t* data, uint_t size);
        bool_t addTrackData(uint8_t trackId, const void* data, uint_t size, uint8_t dataType, bool_t canSkip);
        bool_t close();
        void setMaxFileSize(uint32_t maxSize);
        uint_t recordCount() const { return m_file.records.size(); }
        uint64_t timeMs() const { return m_file.timeMs; }
        uint32_t durationMs() const { return m_file.durationMs; }

    protected:
        LogFile m_file;

    private:
        std::string m_filename;
        uint_t m_fileCount;
        uint_t m_maxFileSize;
    };
}

//--------------------------------------------------------------------------------------------------
#endif
