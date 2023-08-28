#pragma once

#include "nano_fmm/types.hpp"

namespace nano_fmm
{
struct ProjectedPoint
{
    ProjectedPoint(const Eigen::Vector3d &position = {0.0, 0.0, 0.0}, //
                   double distance = 0.0,                             //
                   int64_t road_id = 0, double offset = 0.0)
        : position_(position), distance_(distance), //
          road_id_(road_id), offset_(offset_)
    {
    }
    Eigen::Vector3d position_;
    Eigen::Vector3d direction_;
    double distance_;
    int64_t road_id_;
    double offset_;
};
} // namespace nano_fmm
