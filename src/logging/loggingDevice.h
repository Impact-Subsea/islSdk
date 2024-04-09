#ifndef LOGGINGDEVICE_H_
#define LOGGINGDEVICE_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"
#include "logWriter.h"
#include <memory>

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    class LoggingDevice
    {
    public:
        enum class Type { Unknown, Isl, Nmea };
        enum class LoggingDataType { Unknown, packetData, LogData };

        typedef std::shared_ptr<LoggingDevice> SharedPtr;
        LoggingDevice() : m_trackId(0), m_active(false) {}
        virtual ~LoggingDevice() {}
        void setLogger(std::shared_ptr<LogWriter>& logger, const uint8_t* trackData, uint_t size);
        bool_t log(const uint8_t* data, uint_t size, uint8_t dataType, bool_t canSkip = true);
        virtual bool_t startLogging();
        virtual void stopLogging();
        virtual void newPacketEvent(const uint8_t* data, uint_t size) = 0;
        virtual void logData(uint8_t dataType, const std::vector<uint8_t> data) {};
        bool_t islogging() const { return m_active && m_logger; }

    protected:
        virtual Type getTrackData(std::vector<uint8_t>& buf) = 0;

    private:
        std::shared_ptr<LogWriter> m_logger;
        uint8_t m_trackId;
        bool_t m_active;
    };
}

//--------------------------------------------------------------------------------------------------
#endif
