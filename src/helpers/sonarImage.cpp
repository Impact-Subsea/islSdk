//------------------------------------------ Includes ----------------------------------------------

#include "sonarImage.h"
#include "maths/maths.h"

using namespace IslSdk;

//--------------------------------------------------------------------------------------------------
SonarImage::SonarImage() : m_width(0), m_height(0), m_bpp(0), m_minRangeMm(0), m_maxRangeMm(0), useBilinerInterpolation(true)
{
    setSectorArea(0, 10000, 0, Sonar::maxAngle);
}
//--------------------------------------------------------------------------------------------------
SonarImage::SonarImage(int32_t width, int32_t height, bool_t use4BytePixel, bool_t useBilinerInterpolation) : m_width(0),
m_height(0),
m_bpp(0),
m_minRangeMm(0),
m_maxRangeMm(0),
useBilinerInterpolation(useBilinerInterpolation)
{
    setBuffer(width, height, use4BytePixel);
    setSectorArea(0, 10000, 0, Sonar::maxAngle);
}
//--------------------------------------------------------------------------------------------------
SonarImage::~SonarImage()
{
}
//--------------------------------------------------------------------------------------------------
void SonarImage::setBuffer(int32_t width, int32_t height, bool_t use4BytePixel)
{
    uint8_t bpp = static_cast<uint8_t>(use4BytePixel ? 4 : 2);

    if (m_width != width || m_height != height || m_bpp != bpp)
    {
        m_width = width;
        m_height = height;
        m_bpp = bpp;

        m_buf.resize(width * height * bpp);
        m_redraw = true;

        float_t precision = 100000.0f;
        Box box = minBoundingBox(m_sector, precision);
        m_radius = Math::min(m_width / (static_cast<float_t>(box.width) / precision), m_height / (static_cast<float_t>(box.height) / precision));
    }
}
//--------------------------------------------------------------------------------------------------
void SonarImage::setSectorArea(uint_t minRangeMm, uint_t maxRangeMm, uint_t sectorStart, uint_t sectorSize)
{
    if (sectorSize == 0)
    {
        sectorSize = Sonar::maxAngle;
    }

    if (m_minRangeMm != minRangeMm || m_maxRangeMm != maxRangeMm || m_sector.start != sectorStart || m_sector.size != sectorSize)
    {
        m_minRangeMm = minRangeMm;
        m_maxRangeMm = maxRangeMm;
        m_sector.start = sectorStart;
        m_sector.size = sectorSize;
        m_redraw = true;

        float_t precision = 100000.0f;
        Box box = minBoundingBox(m_sector, precision);
        m_radius = Math::min(m_width / (static_cast<float_t>(box.width) / precision), m_height / (static_cast<float_t>(box.height) / precision));
    }
}
//--------------------------------------------------------------------------------------------------
void SonarImage::render(SonarDataStore& data, const Palette& palette, bool_t reDraw)
{
    Sonar::Sector sector = data.sector;

    if (reDraw || m_redraw)
    {
        m_redraw = false;
        sector = m_sector;
        std::fill(m_buf.begin(), m_buf.end(), palette.data[0x10000].val);
    }
    else
    {
        cropSector(m_sector, sector);
    }

    if (sector.size && m_bpp == 4)
    {
        Box box = minBoundingBox(sector, m_radius);
        float_t radius2 = m_radius * m_radius;

        for (int32_t y = box.y; y < box.height + box.y; y++)
        {
            uint32_t* img = reinterpret_cast<uint32_t*>(&m_buf[((box.height - 1) - (y - box.y)) * m_width * 4]);

            for (int32_t x = box.x; x < box.width + box.x; x++)
            {
                float_t rad2 = static_cast<float_t>(x * x + y * y);

                if (rad2 < radius2)
                {
                    float_t angle = Math::atan2(static_cast<float_t>(x), static_cast<float_t>(y)) * static_cast<float_t>(Sonar::maxAngle / Math::pi2);
                    if (angle < 0.0f)
                    {
                        angle += Sonar::maxAngle;
                    }

                    int32_t dif = static_cast<int32_t>(angle - sector.start);
                    if (dif < 0)
                    {
                        dif += Sonar::maxAngle;
                    }

                    if (dif <= sector.size)
                    {
                        RenderData renderData = getData(data, angle, m_radius);
                        uint32_t pix = 0x10000;
                        if (renderData.row1)
                        {
                            pix = calculatePixel(renderData, std::sqrt(rad2));
                        }
                        *img = palette.data[pix].val;
                    }
                }
                img++;
            }
        }
        data.renderComplete();
    }
}
//--------------------------------------------------------------------------------------------------
void SonarImage::render16Bit(SonarDataStore& data, bool_t reDraw)
{
    Sonar::Sector sector = data.sector;

    if (reDraw || m_redraw)
    {
        m_redraw = false;
        sector = m_sector;
        std::fill(m_buf.begin(), m_buf.end(), 0);
    }
    else
    {
        cropSector(m_sector, sector);
    }

    if (sector.size)
    {
        Box box = minBoundingBox(sector, m_radius);
        float_t radius2 = m_radius * m_radius;

        for (int32_t y = box.y; y < box.height + box.y; y++)
        {
            uint16_t* img = reinterpret_cast<uint16_t*>(&m_buf[((box.height - 1) - (y - box.y)) * m_width * 2]);

            for (int32_t x = box.x; x < box.width + box.x; x++)
            {
                float_t rad2 = static_cast<float_t>(x * x + y * y);

                if (rad2 < radius2)
                {
                    float_t angle = Math::atan2(static_cast<float_t>(x), static_cast<float_t>(y)) * static_cast<float_t>(Sonar::maxAngle / Math::pi2);
                    if (angle < 0.0f)
                    {
                        angle += Sonar::maxAngle;
                    }

                    int32_t dif = static_cast<int32_t>(angle - sector.start);
                    if (dif < 0)
                    {
                        dif += Sonar::maxAngle;
                    }

                    if (dif <= sector.size)
                    {
                        RenderData renderData = getData(data, angle, m_radius);
                        uint16_t pix = 0;
                        if (renderData.row1)
                        {
                            pix = calculatePixel(renderData, Math::sqrt(rad2));
                        }

                        *img = pix;
                    }
                }
                img++;
            }
        }
        data.renderComplete();
    }
}
//--------------------------------------------------------------------------------------------------
bool_t SonarImage::renderTexture(SonarDataStore& data, const Palette& palette, bool_t reDraw)
{
    bool_t modified = false;
    Sonar::Sector sector = data.sector;

    if (reDraw || m_redraw)
    {
        m_redraw = false;
        sector = m_sector;
        std::fill(m_buf.begin(), m_buf.end(), palette.data[0x10000].val);
    }
    else
    {
        cropSector(m_sector, sector);
    }

    if (sector.size && m_bpp == 4)
    {
        int_t secDif = static_cast<int_t>(sector.start - m_sector.start);
        if (secDif < 0) secDif += Sonar::maxAngle;

        float_t yScale = m_height / (static_cast<float_t>(m_sector.size));
        float_t yf = (yScale * secDif) + 0.5f;
        int32_t y = static_cast<int32_t>(yf);
        float_t offset = (yf - y) * yScale + m_sector.start;
        int32_t yCount = static_cast<int32_t>((yScale * sector.size) + 0.5f);
        modified = yCount != 0;
        yScale = 1.0f / yScale;

        uint32_t* img = reinterpret_cast<uint32_t*>(&m_buf[y * m_width * 4]);

        while (yCount--)
        {
            float_t angle = y * yScale + offset;
            if (angle >= data.pingData.size()) angle -= data.pingData.size();
            if (++y > m_height)
            {
                y = 0;
                img = reinterpret_cast<uint32_t*>(&m_buf[0]);
            }

            RenderData renderData = getData(data, angle, static_cast<float_t>(m_width));

            if (renderData.row1)
            {
                for (int32_t x = 0; x < m_width; x++)
                {
                    uint32_t pix = calculatePixel(renderData, static_cast<float_t>(x));
                    *img++ = palette.data[pix].val;
                }
            }
            else
            {
                for (int32_t x = 0; x < m_width; x++)
                {
                    *img++ = palette.data[0x10000].val;
                }
            }
        }

        data.renderComplete();
    }
    return modified;
}
//--------------------------------------------------------------------------------------------------
bool_t SonarImage::renderTexture16Bit(SonarDataStore& data, bool_t reDraw)
{
    bool_t modified = false;
    Sonar::Sector sector = data.sector;

    if (reDraw || m_redraw)
    {
        m_redraw = false;
        sector = m_sector;
        std::fill(m_buf.begin(), m_buf.end(), 0);
    }
    else
    {
        cropSector(m_sector, sector);
    }

    if (sector.size)
    {
        int_t secDif = static_cast<int_t>(sector.start - m_sector.start);
        if (secDif < 0) secDif += Sonar::maxAngle;

        float_t yScale = m_height / (static_cast<float_t>(m_sector.size));
        float_t yf = (yScale * secDif) + 0.5f;
        int32_t y = static_cast<int32_t>(yf);
        float_t offset = (yf - y) * yScale + m_sector.start;
        int32_t yCount = static_cast<int32_t>((yScale * sector.size) + 0.5f);
        modified = yCount != 0;
        yScale = 1.0f / yScale;

        uint16_t* img = reinterpret_cast<uint16_t*>(&m_buf[y * m_width * 2]);

        while (yCount--)
        {
            float_t angle = y * yScale + offset;
            if (angle >= data.pingData.size()) angle -= data.pingData.size();
            if (++y > m_height)
            {
                y = 0;
                img = reinterpret_cast<uint16_t*>(&m_buf[0]);
            }

            RenderData renderData = getData(data, angle, static_cast<float_t>(m_width));

            if (renderData.row1)
            {
                for (int32_t x = 0; x < m_width; x++)
                {
                    *img++ = calculatePixel(renderData, static_cast<float_t>(x));
                }
            }
            else
            {
                Mem::memset(img, 0, m_width * 2);
                img += m_width;
            }
        }

        data.renderComplete();
    }
    return modified;
}
//--------------------------------------------------------------------------------------------------
uint16_t SonarImage::calculatePixel(const RenderData& renderData, float_t range) const
{
    float_t idx = (range * renderData.scale1) + renderData.offset1;
    int_t intIdx = static_cast<int_t>(idx);
    float_t pix = 0.0f;

    if (intIdx >= 0 && intIdx < static_cast<int_t>(renderData.row1->data.size()))
    {
        float_t w = idx - intIdx;

        pix = renderData.row1->data[intIdx] * (1.0f - w);
        if (intIdx < static_cast<int_t>(renderData.row1->data.size() - 1))
        {
            pix += renderData.row1->data[intIdx + 1] * w;
        }
    }

    if (renderData.row2)
    {
        idx = (range * renderData.scale2) + renderData.offset2;
        intIdx = static_cast<int_t>(idx);

        if (intIdx >= 0 && intIdx < static_cast<int_t>(renderData.row2->data.size()))
        {
            float_t w = idx - intIdx;

            float_t pix2 = renderData.row2->data[intIdx] * (1.0f - w);
            if (intIdx < static_cast<int_t>(renderData.row2->data.size() - 1))
            {
                pix2 += renderData.row2->data[intIdx + 1] * w;
            }

            pix = pix * (1.0f - renderData.w) + pix2 * renderData.w;
        }
    }
    return static_cast<uint16_t>(pix);
}
//--------------------------------------------------------------------------------------------------
SonarImage::RenderData SonarImage::getData(const SonarDataStore& data, float_t angle, float_t width) const
{
    RenderData ret;

    ret.row1 = data.pingData[static_cast<uint_t>(angle)];

    if (ret.row1)
    {
        float_t imgRange = static_cast<float_t>(static_cast<int_t>(m_maxRangeMm - m_minRangeMm));

        if (useBilinerInterpolation)
        {
            float_t dif = angle - static_cast<float_t>(ret.row1->angle);
            if (dif > static_cast<float_t>(Sonar::maxAngle / 2))
            {
                dif -= Sonar::maxAngle;
            }
            else if (dif < -static_cast<float_t>(Sonar::maxAngle / 2))
            {
                dif += Sonar::maxAngle;
            }

            if (dif >= 0.0f)
            {
                ret.row2 = data.pingData[ret.row1->endIdx];
            }
            else if (data.pingData[ret.row1->startIdx])
            {
                ret.row2 = ret.row1;
                ret.row1 = data.pingData[ret.row1->startIdx];
            }

            if (ret.row2)
            {
                float_t range = static_cast<float_t>(static_cast<int_t>(ret.row2->maxRangeMm - ret.row2->minRangeMm));
                float_t minDif = static_cast<float_t>(static_cast<int_t>(m_minRangeMm - ret.row2->minRangeMm));
                ret.scale2 = (ret.row2->data.size() / width) * (imgRange / range);
                ret.offset2 = (ret.row2->data.size() / range) * minDif;

                float_t dif1 = Math::abs(angle - static_cast<float_t>(ret.row1->angle));
                if (dif1 > (Sonar::maxAngle / 2))
                {
                    dif1 = Sonar::maxAngle - dif1;
                }

                float_t dif2 = Math::abs(ret.row1->angle - static_cast<float_t>(ret.row2->angle));
                if (dif2 > (Sonar::maxAngle / 2))
                {
                    dif2 = Sonar::maxAngle - dif2;
                }
                ret.w = dif1 / dif2;
            }
        }
        float_t range = static_cast<float_t>(static_cast<int_t>(ret.row1->maxRangeMm - ret.row1->minRangeMm));
        float_t minDif = static_cast<float_t>(static_cast<int_t>(m_minRangeMm - ret.row1->minRangeMm));
        ret.scale1 = (ret.row1->data.size() / width) * (imgRange / range);
        ret.offset1 = (ret.row1->data.size() / range) * minDif;
    }
    return ret;
}
//--------------------------------------------------------------------------------------------------
void SonarImage::cropSector(const Sonar::Sector& window, Sonar::Sector& sector) const
{
    int_t dif = static_cast<int_t>(sector.start - window.start);

    if (dif < 0)
    {
        dif += Sonar::maxAngle;
    }

    if (dif >= static_cast<int_t>(window.size))
    {
        if ((Sonar::maxAngle - dif) < sector.size)
        {
            sector.start = window.start;
            sector.size -= Sonar::maxAngle - dif;
            if (sector.size > window.size)
            {
                sector.size = window.size;
            }
        }
        else
        {
            sector.size = 0;
        }
    }
    else
    {
        uint_t size = sector.size + dif;
        if (size >= Sonar::maxAngle)
        {
            size -= Sonar::maxAngle;
        }

        if (size > window.size)
        {
            sector.size -= size - window.size;
        }
    }
}
//--------------------------------------------------------------------------------------------------
SonarImage::Box SonarImage::minBoundingBox(const Sonar::Sector& sector, float_t radius) const
{
    const float_t a90 = Sonar::maxAngle / 4;
    const float_t a180 = Sonar::maxAngle / 2;
    const float_t a270 = a180 + a90;
    const float_t a360 = Sonar::maxAngle;

    float_t angleStart = static_cast<float_t>(sector.start);
    float_t angleEnd = static_cast<float_t>(sector.start + sector.size);
    float_t startX = Math::sin(angleStart * static_cast<float_t>(Math::pi2 / Sonar::maxAngle)) * radius;
    float_t startY = Math::cos(angleStart * static_cast<float_t>(Math::pi2 / Sonar::maxAngle)) * radius;
    float_t endX = Math::sin(angleEnd * static_cast<float_t>(Math::pi2 / Sonar::maxAngle)) * radius;
    float_t endY = Math::cos(angleEnd * static_cast<float_t>(Math::pi2 / Sonar::maxAngle)) * radius;
    float_t xMax = Math::max(startX, endX);
    float_t xMin = Math::min(startX, endX);
    float_t yMax = Math::max(startY, endY);
    float_t yMin = Math::min(startY, endY);

    if ((angleStart <= a90 && angleEnd >= a90) || (angleStart > a90 && angleEnd >= (a360 + a90)))
    {
        xMax = radius;
    }
    else if (startX <= 0 && endX <= 0)
    {
        xMax = 0;
    }

    if ((angleStart <= a180 && angleEnd >= a180) || (angleStart > a180 && angleEnd >= (a360 + a180)))
    {
        yMin = -radius;
    }
    else if (startY >= 0 && endY >= 0)
    {
        yMin = 0;
    }

    if ((angleStart <= a270 && angleEnd >= a270) || (angleStart > a270 && angleEnd >= (a360 + a270)))
    {
        xMin = -radius;
    }
    else if (startX >= 0 && endX >= 0)
    {
        xMin = 0;
    }

    if (angleEnd >= a360)
    {
        yMax = radius;
    }
    else if (startY <= 0 && endY <= 0)
    {
        yMax = 0;
    }

    Box box(static_cast<int32_t>(xMin), static_cast<int32_t>(yMin), static_cast<int32_t>(xMax - xMin), static_cast<int32_t>(yMax - yMin));

    return box;
}
//--------------------------------------------------------------------------------------------------
/*void SonarImage::renderTextureFromPing(const Sonar::Ping& ping, const Palette& palette)
{
    int_t stepSize = ping.stepSize == 0 ? 64 : ping.stepSize;
    uint_t yStart = static_cast<uint_t>((m_height / static_cast<float_t>(m_sector.size)) * ping.angle);
    uint_t yCount = 1 + static_cast<uint_t>((m_height / static_cast<float_t>(m_sector.size)) * Math::abs(stepSize));

    float_t xScale = (ping.data.size() / static_cast<float_t>(ping.maxRangeMm - ping.minRangeMm)) * (static_cast<float_t>(m_maxRangeMm - m_minRangeMm)) / m_width;
    float_t xOffset = (m_minRangeMm - ping.minRangeMm) * xScale;

    uint32_t* img = reinterpret_cast<uint32_t*>(&m_buf[yStart * m_width * 4]);
    if (yStart == 0)
    {
        yStart = m_height;
    }
    uint32_t* prev = reinterpret_cast<uint32_t*>(&m_buf[(yStart - 1) * m_width * 4]);

    for (uint_t y = 0; y < yCount; y++)
    {
        float_t wy = (1.0f / yCount) * y;

        for (uint_t x = 0; x < m_width; x++)
        {
            uint32_t pix = palette.nullColour.val;
            float_t idx = (x * xScale) + xOffset;
            int_t intIdx = static_cast<int_t>(idx);

            if (intIdx >= 0 && intIdx < ping.data.size())
            {
                float_t w = idx - intIdx;

                pix = ping.data[intIdx] * (1.0f - w);
                if (intIdx < ping.data.size() - 1)
                {
                    pix += ping.data[intIdx + 1] * w;
                }
                pix = palette.data[pix & 0xffff].val;
                if (m_useBilinerInterpolation)
                {
                    pix = Palette::Colour::interpolate(prev[x], pix, wy).val;
                }
            }
            *img++ = pix;
        }
    }
}*/
//--------------------------------------------------------------------------------------------------
