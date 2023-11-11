#ifndef SONARIMAGE_H_
#define SONARIMAGE_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"
#include "devices/sonar.h"
#include "sonarDataStore.h"
#include "palette.h"
#include <vector>

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    class SonarImage
    {
    public:
        const std::vector<uint8_t>& buf = m_buf;        ///< The pixel buffer
        const int32_t& width = m_width;                 ///< Width of the buffer in pixels
        const int32_t& height = m_height;               ///< Height of the buffer in pixels
        const uint8_t& bpp = m_bpp;
        bool_t useBilinerInterpolation;

        SonarImage();
        SonarImage(int32_t width, int32_t height, bool_t use4BytePixel = true, bool_t useBilinerInterpolation = true);
        ~SonarImage();
        void setBuffer(int32_t width, int32_t height, bool_t use4BytePixel);
        void setSectorArea(uint_t minRangeMm, uint_t maxRangeMm, uint_t sectorStart, uint_t sectorSize);

        void render(SonarDataStore& data, const Palette& palette, bool_t reDraw = false);
        void render16Bit(SonarDataStore& data, bool_t reDraw = false);
        bool_t renderTexture(SonarDataStore& data, const Palette& palette, bool_t reDraw = false);
        bool_t renderTexture16Bit(SonarDataStore& data, bool_t reDraw = false);

    private:
        struct RenderData
        {
            SonarDataStore::SharedPtr row1;
            SonarDataStore::SharedPtr row2;
            float_t scale1;
            float_t offset1;
            float_t scale2;
            float_t offset2;
            float_t w;
            RenderData() : scale1(0), offset1(0), scale2(0), offset2(0), w(0) { }
        };

        struct Box
        {
            int32_t x;
            int32_t y;
            int32_t width;
            int32_t height;
            Box(int32_t x, int32_t y, int32_t width, int32_t height) : x(x), y(y), width(width), height(height) { }
        };

        uint16_t calculatePixel(const RenderData& renderData, float_t range) const;
        RenderData getData(const SonarDataStore& data, float_t angle, float_t width) const;
        void cropSector(const Sonar::Sector& window, Sonar::Sector& sector) const;
        Box minBoundingBox(const Sonar::Sector& sector, float_t radius) const;

        std::vector<uint8_t> m_buf;             ///< The pixel buffer
        int32_t m_width;                        ///< Width of the buffer in pixels
        int32_t m_height;                       ///< Height of the buffer in pixels
        uint8_t m_bpp;
        uint_t m_minRangeMm;                    ///< Lower range in millimeters
        uint_t m_maxRangeMm;                    ///< Upper range in millimeters
        Sonar::Sector m_sector;
        bool_t m_redraw;
        float_t m_radius;
    };
}

//--------------------------------------------------------------------------------------------------
#endif
