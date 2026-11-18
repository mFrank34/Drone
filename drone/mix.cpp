#include "mix.h"

namespace mix
{
    void stabilised(float base, Correction c, float outs[CORNER_COUNT])
    {
        outs[CORNER_FL] = base - c.roll + c.pitch;
        outs[CORNER_FR] = base + c.roll + c.pitch;
        outs[CORNER_BL] = base - c.roll - c.pitch;
        outs[CORNER_BR] = base + c.roll - c.pitch;
    }

    void all(float base, float outs[CORNER_COUNT])
    {
        for (uint8_t i = 0; i < CORNER_COUNT; i++)
        {
            outs[i] = base;
        }
    }

    // sel is 1..4 (FL FR BL BR). Out-of-range values leave outs as zero.
    void single(float base, uint8_t sel, float outs[CORNER_COUNT])
    {
        // Zero out the array first so out-of-range values result in all zeros
        for (uint8_t i = 0; i < CORNER_COUNT; i++)
        {
            outs[i] = 0.0f;
        }

        if (sel >= 1 && sel <= CORNER_COUNT)
        {
            outs[sel - 1] = base;
        }
    }

} // namespace mix