#pragma once

namespace nano_fmm
{
struct Network
{
    Network(bool is_wgs84 = true) : is_wgs84_(is_wgs84) {}

  private:
    const bool is_wgs84_{true};
};
} // namespace nano_fmm
