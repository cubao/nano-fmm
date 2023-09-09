#pragma once

#include "nano_fmm/types.hpp"

namespace nano_fmm
{
struct Config
{

    SETUP_FLUENT_API(Config, double, ubodt_thresh)
    Config &from_rapidjson(const RapidjsonValue &json);
    RapidjsonValue to_rapidjson(RapidjsonAllocator &allocator) const;
    RapidjsonValue to_rapidjson() const
    {
        RapidjsonAllocator allocator;
        return to_rapidjson(allocator);
    }

  private:
    double ubodt_thresh_ = 3000.0;
};
} // namespace nano_fmm
