#pragma once

#include "nano_fmm/types.hpp"

namespace nano_fmm
{
struct ProjectedPoint
{
    ProjectedPoint() {}
    ProjectedPoint(const Eigen::Vector3d &position,
                   const Eigen::Vector3d &direction, //
                   double distance, int64_t road_id,
                   double offset)
        : position_(position),   //
          direction_(direction), //
          distance_(distance),   //
          road_id_(road_id),     //
          offset_(offset)
    {
    }

    SETUP_FLUENT_API(ProjectedPoint, Eigen::Vector3d, position)
    SETUP_FLUENT_API(ProjectedPoint, Eigen::Vector3d, direction)
    SETUP_FLUENT_API(ProjectedPoint, double, distance)
    SETUP_FLUENT_API(ProjectedPoint, int64_t, road_id)
    SETUP_FLUENT_API(ProjectedPoint, double, offset)
    ProjectedPoint &from_rapidjson(const RapidjsonValue &json);
    RapidjsonValue to_rapidjson(RapidjsonAllocator &allocator) const;
    RapidjsonValue to_rapidjson() const
    {
        RapidjsonAllocator allocator;
        return to_rapidjson(allocator);
    }

  private:
    Eigen::Vector3d position_{0.0, 0.0, 0.0};
    Eigen::Vector3d direction_{0.0, 0.0, 1.0};
    double distance_{0.0};
    int64_t road_id_{0};
    double offset_{0.0};
};
} // namespace nano_fmm
