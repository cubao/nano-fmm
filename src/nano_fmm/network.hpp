#pragma once

#include "nano_fmm/types.hpp"
#include "nano_fmm/config.hpp"
#include "nano_fmm/polyline.hpp"

#include <optional>
#include <memory>
#include <vector>
#include <unordered_map>
#include <unordered_set>

namespace nano_fmm
{
struct ProjectedPoint
{
    Eigen::Vector3d position_;
    double distance_;
    int64_t road_id_;
    double offset_;
};

struct Network
{
    Network(bool is_wgs84 = false) : is_wgs84_(is_wgs84) {}

    std::shared_ptr<Config> config();
    void config(std::shared_ptr<Config> config);

    void add_road(const Eigen::Ref<RowVectors> &geom, int64_t road_id);
    void add_link(int64_t source_road, int64_t target_road);
    void remove_road(int64_t road_id);
    void remove_link(int64_t source_road, int64_t target_road);
    std::unordered_set<int64_t> prev_roads(int64_t road_id) const;
    std::unordered_set<int64_t> next_roads(int64_t road_id) const;
    std::unordered_set<int64_t> roads() const;

    std::vector<ProjectedPoint> query(const Eigen::Vector3d &position,
                                      double radius,
                                      std::optional<int> k = std::nullopt);

    static Network load(const std::string &path);
    bool dump(const std::string &path) const;

    bool build_ubodt(std::optional<double> thresh) const;

    Network to_2d() const;

  private:
    const bool is_wgs84_{false};
    // roads (id -> geom)
    std::unordered_map<int64_t, Polyline> roads_;
    // links (id -> {nexts}, id -> {prevs})
    std::unordered_map<int64_t, std::unordered_set<int64_t>> nexts_, prevs_;
    // config
    std::shared_ptr<Config> config_;
};
} // namespace nano_fmm
