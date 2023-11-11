#ifndef SONARDATASTORE_H_
#define SONARDATASTORE_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"
#include "devices/sonar.h"
#include <vector>
#include <array>

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    class SonarDataStore
    {
    public:
        SonarDataStore();
        ~SonarDataStore();

        struct PingData
        {
            uint_t startIdx;
            uint_t endIdx;
            uint_t minRangeMm;
            uint_t maxRangeMm;
            uint_t angle;
            std::vector<uint16_t> data;

            PingData() : startIdx(0), endIdx(0), minRangeMm(0), maxRangeMm(0), angle(0) {}
        };

        typedef std::shared_ptr<PingData> SharedPtr;

        const Sonar::Sector& sector = m_sector;
        const std::array<SharedPtr, Sonar::maxAngle>& pingData = m_pingData;

        bool_t add(const Sonar::Ping& ping, uint_t blankRangeMm = 0);
        void clear(uint_t startAngle = 0, int_t angleSize = Sonar::maxAngle);
        void renderComplete() { m_resetSector = true; }

    private:
        std::array<SharedPtr, Sonar::maxAngle> m_pingData;
        SonarDataStore::SharedPtr clearPingData(uint_t startAngle, uint_t angleSize);
        void updateSector(uint_t angle, uint_t sectorSize);
        inline uint_t wrap(uint_t v1, int_t v2) const;
        inline uint_t distance(uint_t v1, uint_t v2) const;
        bool_t m_resetSector;
        Sonar::Sector m_sector;
    };
}

//--------------------------------------------------------------------------------------------------
#endif
