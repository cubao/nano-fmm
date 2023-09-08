#pragma once

#include "nano_fmm/types.hpp"

namespace nano_fmm
{
struct UbodtRecord
{
    UbodtRecord() {}
    UbodtRecord(int64_t source_road, int64_t target_road, //
                int64_t source_next, int64_t target_prev, //
                double cost, UbodtRecord *next = nullptr)
        : source_road_(source_road), target_road_(target_road), //
          source_next_(source_next), target_prev_(target_prev), //
          cost_(cost), next_(next)
    {
    }

    int64_t source_road_{0};
    int64_t target_road_{0};
    int64_t source_next_{0};
    int64_t target_prev_{0};
    double cost_{0.0};
    UbodtRecord *next_{nullptr};

    bool operator<(const UbodtRecord &rhs) const
    {
        if (source_road_ != rhs.source_road_) {
            return source_road_ < rhs.source_road_;
        }
        if (cost_ != rhs.cost_) {
            return cost_ < rhs.cost_;
        }
        if (source_next_ != rhs.source_next_) {
            return source_next_ < rhs.source_next_;
        }
        return std::make_tuple(target_prev_, target_road_, next_) <
               std::make_tuple(rhs.target_prev_, rhs.target_road_, rhs.next_);
    }
    bool operator==(const UbodtRecord &rhs) const
    {
        return source_road_ == rhs.source_road_ &&
               target_road_ == rhs.target_road_ &&
               source_next_ == rhs.source_next_ &&
               target_prev_ == rhs.target_prev_ && next_ == rhs.next_;
    }
};

} // namespace nano_fmm
