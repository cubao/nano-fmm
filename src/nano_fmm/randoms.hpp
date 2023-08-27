#pragma

#include <random>
#include <algorithm>

namespace nano_fmm
{
using RGB = std::array<uint8_t, 3>;
inline RGB hsv_to_rgb(float h, float s, float v)
{
    float r, g, b;
    int i = std::floor(h * 6);
    float f = h * 6 - i;
    float p = v * (1 - s);
    float q = v * (1 - f * s);
    float t = v * (1 - (1 - f) * s);

    switch (i % 6) {
    case 0:
        r = v, g = t, b = p;
        break;
    case 1:
        r = q, g = v, b = p;
        break;
    case 2:
        r = p, g = v, b = t;
        break;
    case 3:
        r = p, g = q, b = v;
        break;
    case 4:
        r = t, g = p, b = v;
        break;
    case 5:
        r = v, g = p, b = q;
        break;
    }
    return {static_cast<uint8_t>(r * 255), //
            static_cast<uint8_t>(g * 255), //
            static_cast<uint8_t>(b * 255)};
}

struct RandomStroke
{
    RandomStroke(bool on_black = true) : on_black_(on_black)
    {
        std::random_device rd;
        mt_ = std::mt19937(rd());
    }
    RandomStroke(int seed, bool on_black = true) : on_black_(on_black)
    {
        mt_ = std::mt19937(seed);
    }
    RGB next()
    {
        float h = std::uniform_real_distribution<float>(0.0f, 1.f)(mt_);
        float s = std::uniform_real_distribution<float>(0.4f, 1.f)(mt_);
        float v = std::uniform_real_distribution<float>(0.7f, 1.f)(mt_);
        if (!on_black_) {
            s = 1.f - s;
            v = 1.f - v;
        }
        return hsv_to_rgb(h, s, v);
    }

  private:
    // random stroke good for black/dark background
    // else good for white/bright background
    const bool on_black_{true};
    std::mt19937 mt_;
};
} // namespace nano_fmm
