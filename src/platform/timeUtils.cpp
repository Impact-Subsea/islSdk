//------------------------------------------ Includes ----------------------------------------------

#include "timeUtils.h"
#include <chrono>

using namespace IslSdk;

static int64_t timeCorrectionMs = 0;

//--------------------------------------------------------------------------------------------------
int64_t Time::getCorrectionMs()
{
    return timeCorrectionMs;
}
//--------------------------------------------------------------------------------------------------
void Time::resetCorrection()
{
    timeCorrectionMs = 0;
}
//--------------------------------------------------------------------------------------------------
uint64_t Time::getTimeMs()
{
    return getSystemTimeMs() - timeCorrectionMs;
}
//--------------------------------------------------------------------------------------------------
int64_t Time::getSystemTimeMs()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}
//--------------------------------------------------------------------------------------------------
void Time::set(int_t year, int_t month, int_t day, int_t hour, int_t minute, real_t second)
{
    int64_t now = getSystemTimeMs();

    std::tm tm_time = {};
    tm_time.tm_year = static_cast<int32_t>(year - 1900);
    tm_time.tm_mon = static_cast<int32_t>(month - 1);
    tm_time.tm_mday = static_cast<int32_t>(day);
    tm_time.tm_hour = static_cast<int32_t>(hour);
    tm_time.tm_min = static_cast<int32_t>(minute);
    tm_time.tm_sec = static_cast<int32_t>(second);

    std::chrono::system_clock::time_point tp = std::chrono::system_clock::from_time_t(std::mktime(&tm_time));
    tp += std::chrono::milliseconds(static_cast<int_t>(second - static_cast<int_t>(second)));

    timeCorrectionMs = now - std::chrono::duration_cast<std::chrono::milliseconds>(tp.time_since_epoch()).count();
}
//--------------------------------------------------------------------------------------------------
