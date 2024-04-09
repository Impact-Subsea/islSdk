//------------------------------------------ Includes ----------------------------------------------

#include "palette.h"

using namespace IslSdk;

//--------------------------------------------------------------------------------------------------
Palette::Colour::Colour() : val(0)
{
}
//--------------------------------------------------------------------------------------------------
Palette::Colour::Colour(uint32_t colour) : val(colour)
{
}
//--------------------------------------------------------------------------------------------------
Palette::Colour::Colour(uint8_t r, uint8_t g, uint8_t b, uint8_t a) : b{ r, g, b, a }
{
}
//--------------------------------------------------------------------------------------------------
Palette::Colour Palette::Colour::interpolate(Colour to, float_t weight) const
{
    return {
        static_cast<uint8_t>(static_cast<float_t>(b[0]) * (1.0f - weight) + static_cast<float_t>(to.b[0]) * weight),
        static_cast<uint8_t>(static_cast<float_t>(b[1]) * (1.0f - weight) + static_cast<float_t>(to.b[1]) * weight),
        static_cast<uint8_t>(static_cast<float_t>(b[2]) * (1.0f - weight) + static_cast<float_t>(to.b[2]) * weight),
        static_cast<uint8_t>(static_cast<float_t>(b[3]) * (1.0f - weight) + static_cast<float_t>(to.b[3]) * weight) };
}
//--------------------------------------------------------------------------------------------------
Palette::Colour Palette::Colour::interpolate(Colour from, Colour to, float_t weight)
{
    return {
        static_cast<uint8_t>(static_cast<float_t>(from.b[0]) * (1.0f - weight) + static_cast<float_t>(to.b[0]) * weight),
        static_cast<uint8_t>(static_cast<float_t>(from.b[1]) * (1.0f - weight) + static_cast<float_t>(to.b[1]) * weight),
        static_cast<uint8_t>(static_cast<float_t>(from.b[2]) * (1.0f - weight) + static_cast<float_t>(to.b[2]) * weight),
        static_cast<uint8_t>(static_cast<float_t>(from.b[3]) * (1.0f - weight) + static_cast<float_t>(to.b[3]) * weight) };
}
//--------------------------------------------------------------------------------------------------
Palette::Palette()
{
    setToDefault();
}
//--------------------------------------------------------------------------------------------------
Palette::~Palette()
{
}
//--------------------------------------------------------------------------------------------------
void Palette::setToDefault()
{
    std::vector<GradientValue> palette;
    palette.emplace_back(0xff000000, static_cast<uint16_t>(static_cast<real_t>(m_data.size() - 2) * 0.0));
    palette.emplace_back(0xffab5a20, static_cast<uint16_t>(static_cast<real_t>(m_data.size() - 2) * 0.22));
    palette.emplace_back(0xff3dce27, static_cast<uint16_t>(static_cast<real_t>(m_data.size() - 2) * 0.4));
    palette.emplace_back(0xff23cec2, static_cast<uint16_t>(static_cast<real_t>(m_data.size() - 2) * 0.62));
    palette.emplace_back(0xff0012ce, static_cast<uint16_t>(static_cast<real_t>(m_data.size() - 2) * 1.0));

    set(palette, 0xff000000);
}
//--------------------------------------------------------------------------------------------------
void Palette::set(const std::vector<GradientValue>& gradient, Colour nullColour)
{
    m_data[m_data.size() - 1] = nullColour;

    if (gradient.size())
    {
        for (uint_t i = 0; i < gradient[0].position; i++)
        {
            m_data[i] = nullColour;
        }

        if (gradient.size() == 1)
        {
            for (uint_t i = gradient[0].position; i < (m_data.size() - 1); i++)
            {
                m_data[i] = gradient[0].colour;
            }
        }
        else
        {
            uint_t endIdx = 0;

            for (uint_t n = 1; n < gradient.size(); n++)
            {
                uint_t startIdx = gradient[n - 1].position;
                endIdx = gradient[n].position;
                float_t scale = 1.0f / (endIdx - startIdx);

                for (uint_t i = startIdx, x = 0; i <= endIdx; i++, x++)
                {
                    m_data[i] = gradient[n - 1].colour.interpolate(gradient[n].colour, scale * x);
                }
            }

            if (endIdx < m_data.size() - 2)
            {
                for (uint_t i = endIdx; i < (m_data.size() - 1); i++)
                {
                    m_data[i] = nullColour;
                }
            }
        }
    }
}
//--------------------------------------------------------------------------------------------------
void Palette::render(uint32_t* buf, uint_t width, uint_t height, bool_t horizontal)
{
    if (horizontal)
    {
        float_t scale = static_cast<float_t>(m_data.size()) / static_cast<float_t>(width);

        for (uint_t i = 0; i < width; i++)
        {
            uint_t idx = static_cast<uint_t>(scale * i);
            buf[i] = m_data[idx].val;
        }

        for (uint_t i = 1; i < height; i++)
        {
            Mem::memcpy(&buf[i * width], &buf[0], width * 4);
        }
    }
    else
    {
        float_t scale = static_cast<float_t>(m_data.size()) / static_cast<float_t>(height);

        for (uint_t i = 0; i < height; i++)
        {
            uint_t idx = static_cast<uint_t>(scale * i);
            for (uint_t x = 0; x < width; x++)
            {
                buf[(height - i - 1) * width + x] = m_data[idx].val;
            }
        }
    }
}
//--------------------------------------------------------------------------------------------------
