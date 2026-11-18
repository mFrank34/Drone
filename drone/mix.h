// mix.h

namespace mix
{
    void stabilised(float base, Correction c, float outs[CORNER_COUNT]);
    void all(float base, float outs[CORNER_COUNT]);
    void single(float base, uint8_t sel, float outs[CORNER_COUNT]);
} // namespace mix
