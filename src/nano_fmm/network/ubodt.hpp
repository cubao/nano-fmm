#pragma once

#include "nano_fmm/types.hpp"

namespace nano_fmm
{
struct UbodtRecord
{
    UbodtRecord() {}
    UbodtRecord(int64_t source_road, int64_t target_road, //
                int64_t source_next, int64_t target_prev, //
                double cost)
        : source_road_(source_road), target_road_(target_road), //
          source_next_(source_next), target_prev_(target_prev), //
          cost_(cost)
    {
    }

    SETUP_FLUENT_API(UbodtRecord, int64_t, source_road)
    SETUP_FLUENT_API(UbodtRecord, int64_t, target_road)
    SETUP_FLUENT_API(UbodtRecord, int64_t, source_next)
    SETUP_FLUENT_API(UbodtRecord, int64_t, target_prev)
    SETUP_FLUENT_API(UbodtRecord, double, cost)
    UbodtRecord &from_rapidjson(const RapidjsonValue &json);
    RapidjsonValue to_rapidjson(RapidjsonAllocator &allocator) const;
    RapidjsonValue to_rapidjson() const
    {
        RapidjsonAllocator allocator;
        return to_rapidjson(allocator);
    }

    bool operator<(const UbodtRecord &rhs) const
    {
        if (source_road_ != rhs.source_road_) {
            return source_road_ < rhs.source_road_;
        }
        if (cost_ != rhs.cost_) {
            return cost_ < rhs.cost_;
        }
        return std::make_tuple(source_next_, target_prev_, target_road_) <
               std::make_tuple(rhs.source_next_, rhs.target_prev_,
                               rhs.target_road_);
    }
    bool operator==(const UbodtRecord &rhs) const
    {
        return source_road_ == rhs.source_road_ &&
               target_road_ == rhs.target_road_ &&
               source_next_ == rhs.source_next_ &&
               target_prev_ == rhs.target_prev_ && cost_ == rhs.cost_;
    }

  private:
    int64_t source_road_{0};
    int64_t target_road_{0};
    int64_t source_next_{0};
    int64_t target_prev_{0};
    double cost_{0.0};
};

} // namespace nano_fmm
