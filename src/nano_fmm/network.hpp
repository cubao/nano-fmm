#pragma once

#include "nano_fmm/types.hpp"
#include "nano_fmm/config.hpp"
#include "nano_fmm/polyline.hpp"

#include <optional>
#include <memory>
#include <vector>
#include <unordered_map>

namespace nano_fmm
{
struct ProjectedPoint
{
    Eigen::Vector3d position_;
    double distance_;
    int64_t edge_id_;
    double edge_along_;
};

struct Network
{
    Network(bool is_wgs84 = false) : is_wgs84_(is_wgs84) {}

    std::shared_ptr<Config> config();
    void config(std::shared_ptr<Config> config);

    void add_node(int64_t node_id, const Eigen::Ref<RowVectors> &polyline);
    void add_edge(int64_t edge_id, int64_t source_node, int64_t target_node);

    std::vector<ProjectedPoint>
    query(const Eigen::Vector3d &position,
          std::optional<double> radius = std::nullopt,
          std::optional<double> k = std::nullopt);

    int load(const std::string &path);
    bool dump(const std::string &path) const;

    bool build_ubodt(std::optional<double> thresh) const;

    Network to_2d() const;

  private:
    bool is_wgs84_{true};
    std::vector<int64_t> ids_;
    std::unordered_map<int64_t, size_t> id2index_;
    std::unordered_map<int64_t, Polyline> polylines_;

    std::shared_ptr<Config> config_;
};
} // namespace nano_fmm
