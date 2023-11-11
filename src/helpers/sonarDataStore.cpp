//------------------------------------------ Includes ----------------------------------------------

#include "sonarDataStore.h"
#include "platform/mem.h"
#include "maths/maths.h"

using namespace IslSdk;

//--------------------------------------------------------------------------------------------------
SonarDataStore::SonarDataStore() : m_resetSector(false)
{
}
//--------------------------------------------------------------------------------------------------
SonarDataStore::~SonarDataStore()
{
}
//--------------------------------------------------------------------------------------------------
bool_t SonarDataStore::add(const Sonar::Ping& ping, uint_t blankRangeMm)
{
    uint_t blank = 0;

    if (ping.stepSize)
    {
        uint_t stepSize = Math::abs(ping.stepSize);
        stepSize = Math::min<uint_t>(stepSize, Sonar::maxAngle / 4);

        if (m_resetSector)
        {
            m_resetSector = false;
            m_sector.size = 0;
        }

        uint_t idx = wrap(ping.angle, static_cast<int_t>(stepSize) / -2);
        std::shared_ptr<PingData> node = clearPingData(idx, stepSize);
        updateSector(idx, stepSize);

        if (node == nullptr)
        {
            node = std::make_shared<PingData>();
        }

        node->startIdx = wrap(idx, -1);
        node->endIdx = wrap(idx, stepSize);
        node->minRangeMm = ping.minRangeMm;
        node->maxRangeMm = ping.maxRangeMm;
        node->angle = ping.angle;
        node->data = ping.data;

        if (blankRangeMm > ping.minRangeMm)
        {
            blank = static_cast<uint_t>(static_cast<float_t>(blankRangeMm - ping.minRangeMm) * (ping.data.size() / static_cast<float_t>(ping.maxRangeMm - ping.minRangeMm)));
            blank = Math::min(blank, ping.data.size());
            std::fill_n(node->data.begin(), blank, 0);
        }

        for (uint_t i = 0; i < stepSize; i++)
        {
            m_pingData[idx] = node;
            idx = wrap(idx, 1);
        }
    }

    return false;
}
//--------------------------------------------------------------------------------------------------
void SonarDataStore::clear(uint_t startAngle, int_t angleSize)
{
    if (angleSize < 0)
    {
        startAngle = wrap(startAngle, angleSize);
        angleSize = -angleSize;
    }

    clearPingData(startAngle, static_cast<uint_t>(angleSize));
}
//--------------------------------------------------------------------------------------------------
std::shared_ptr<SonarDataStore::PingData> SonarDataStore::clearPingData(uint_t startIdx, uint_t angleSize)
{
    std::shared_ptr<PingData> node;

    if (angleSize)
    {
        if (m_pingData[startIdx])
        {
            m_pingData[startIdx]->endIdx = startIdx;
            if (distance(m_pingData[startIdx]->endIdx, m_pingData[startIdx]->angle) > (Sonar::maxAngle / 2))
            {
                m_pingData[startIdx]->angle = wrap(startIdx, -1);
            }
        }

        uint_t endIdx = wrap(startIdx, static_cast<int_t>(angleSize));

        if (m_pingData[endIdx] && m_pingData[startIdx] && (m_pingData[startIdx].get() != m_pingData[endIdx].get()))
        {
            m_pingData[endIdx]->startIdx = wrap(endIdx, -1);
            if (distance(m_pingData[endIdx]->angle, m_pingData[endIdx]->startIdx) > (Sonar::maxAngle / 2))
            {
                m_pingData[startIdx]->angle = endIdx;
            }
        }

        for (uint_t i = 0; i < angleSize; i++)
        {
            if (m_pingData[startIdx])
            {
                if (!node && m_pingData[startIdx].use_count() == 1)
                {
                    node = m_pingData[startIdx];
                }
                m_pingData[startIdx].reset();
            }
            startIdx = wrap(startIdx, 1);
        }
    }
    return node;
}
//--------------------------------------------------------------------------------------------------
void SonarDataStore::updateSector(uint_t angle, uint_t sectorSize)
{
    if (m_sector.size == 0)
    {
        m_sector.start = angle;
        m_sector.size = sectorSize;
    }
    else
    {
        uint_t dif = distance(angle, m_sector.start);

        if (dif > m_sector.size)
        {
            if ((dif - m_sector.size) <= (Sonar::maxAngle - dif))
            {
                m_sector.size += (dif - m_sector.size) + sectorSize;
            }
            else
            {
                m_sector.start = angle;
                m_sector.size += Sonar::maxAngle - dif;
                if (m_sector.size < sectorSize)
                {
                    m_sector.size += sectorSize - m_sector.size;
                }
            }
        }
    }
}
//--------------------------------------------------------------------------------------------------
uint_t SonarDataStore::wrap(uint_t v1, int_t v2) const
{
    int_t v = static_cast<int_t>(v1) + v2;
    if (v >= static_cast<int_t>(Sonar::maxAngle))
    {
        v -= Sonar::maxAngle;
    }
    else if (v < 0)
    {
        v += Sonar::maxAngle;
    }
    return static_cast<uint_t>(v);
}
//--------------------------------------------------------------------------------------------------
uint_t SonarDataStore::distance(uint_t v1, uint_t v2) const
{
    int_t v = static_cast<int_t>(v1 - v2);
    if (v >= static_cast<int_t>(Sonar::maxAngle))
    {
        v -= Sonar::maxAngle;
    }
    else if (v < 0)
    {
        v += Sonar::maxAngle;
    }
    return static_cast<uint_t>(v);
}
//--------------------------------------------------------------------------------------------------
