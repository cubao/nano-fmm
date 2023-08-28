#pragma once

#include "nano_fmm/types.hpp"

namespace nano_fmm
{
struct UbodtRecord
{
    int64_t source_road{0};
    int64_t target_road{0};
    int64_t source_next{0};
    int64_t target_prev{0};
    double cost{0.0};
    UbodtRecord *next{nullptr};

    bool operator<(const UbodtRecord &rhs) const
    {
        if (source_road != rhs.source_road) {
            return source_road < rhs.source_road;
        }
        if (cost != rhs.cost) {
            return cost < rhs.cost;
        }
        if (source_next != rhs.source_next) {
            return source_next < rhs.source_next;
        }
        return std::make_tuple(target_prev, target_road, next) <
               std::make_tuple(rhs.target_prev, rhs.target_road, rhs.next);
    }
    bool operator==(const UbodtRecord &rhs) const
    {
        return source_road == rhs.source_road &&
               target_road == rhs.target_road &&
               source_next == rhs.source_next &&
               target_prev == rhs.target_prev && next == rhs.next;
    }
};

} // namespace nano_fmm
