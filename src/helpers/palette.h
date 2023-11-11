#ifndef PALETTE_H_
#define PALETTE_H_

//------------------------------------------ Includes ----------------------------------------------

#include <vector>
#include <array>
#include "devices/sonar.h"

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    class Palette
    {
    public:
        union Colour
        {
            uint32_t val;
            uint8_t b[4];

            Colour();
            Colour(uint32_t c);
            Colour(uint8_t r, uint8_t g, uint8_t b, uint8_t a);
            Colour interpolate(Colour to, float_t weight) const;
            static Colour interpolate(Colour from, Colour to, float_t weight);
        };

        /// A gradient node type
        struct GradientValue
        {
            Colour colour;                ///< Colour value expressed as your application requires. eg ARGB or RGBA
            uint16_t position;            ///< Postion of this colour in the 16 bit pallet 0x0000 to 0xffff
            GradientValue(Colour colour, uint16_t position) : position(position), colour(colour) { }
        };

        Palette();
        ~Palette();
        void setToDefault();
        void set(const std::vector<GradientValue>& gradient, Colour nullColour);
        void render(uint32_t* buf, uint_t width, uint_t height, bool_t horizontal);
        const std::array<Colour, 65536 + 1>& data = m_data;

    private:
        std::array<Colour, 65536 + 1> m_data;
    };
}

//--------------------------------------------------------------------------------------------------
#endif
