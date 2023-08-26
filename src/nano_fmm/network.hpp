#pragma once

#include "nano_fmm/types.hpp"
#include "nano_fmm/config.hpp"
#include "nano_fmm/polyline.hpp"

#include "packedrtree.h"

#include <optional>
#include <memory>
#include <vector>
#include <map>
#include <unordered_map>
#include <unordered_set>

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

struct UbodtRecord
{
    int64_t source_road;
    int64_t target_road;
    int64_t source_next;
    int64_t target_prev;
    double cost;
    UbodtRecord *next;
};

struct Network
{
    Network(bool is_wgs84 = false) : is_wgs84_(is_wgs84) {}

    const Config &config() const { return config_; }
    Config &config() { return config_; }

    void add_road(const Eigen::Ref<RowVectors> &geom, int64_t road_id);
    void add_link(int64_t source_road, int64_t target_road);
    void remove_road(int64_t road_id);
    void remove_link(int64_t source_road, int64_t target_road);
    std::unordered_set<int64_t> prev_roads(int64_t road_id) const;
    std::unordered_set<int64_t> next_roads(int64_t road_id) const;
    std::unordered_set<int64_t> roads() const;

    const Polyline *road(int64_t road_id) const;

    std::vector<ProjectedPoint>
    query(const Eigen::Vector3d &position, double radius,
          std::optional<int> k = std::nullopt,
          std::optional<double> z_max_offset = std::nullopt) const;
    std::map<std::tuple<int64_t, int64_t>, RowVectors>
    query(const Eigen::Vector4d &bbox) const;

    void build() const;

    // graph operations
    // move forward/backward N meters, return ProjectPoint
    // single source dijkstra

    static std::unique_ptr<Network> load(const std::string &path);
    bool dump(const std::string &path, bool with_config = true) const;

    std::vector<UbodtRecord> build_ubodt(std::optional<double> thresh) const;
    bool load_ubodt(const std::string &path);
    bool dump_ubodt(const std::string &path,
                    std::optional<double> thresh) const;

    Network to_2d() const;

  private:
    const bool is_wgs84_{false};
    // roads (id -> geom)
    std::unordered_map<int64_t, Polyline> roads_;
    // links (id -> {nexts}, id -> {prevs})
    std::unordered_map<int64_t, std::unordered_set<int64_t>> nexts_, prevs_;
    // config
    Config config_;

    // spatial index
    mutable std::vector<IndexIJ> segs_;
    mutable std::unordered_map<IndexIJ, size_t, hash_eigen<IndexIJ>> seg2idx_;
    mutable std::optional<FlatGeobuf::PackedRTree> rtree_;
    FlatGeobuf::PackedRTree &rtree() const;

    using IndexMap = std::unordered_map<int64_t, int64_t>;
    using DistanceMap = std::unordered_map<int64_t, double>;
    void single_source_upperbound_dijkstra(int64_t source, double distance, //
                                           IndexMap *predecessor_map,
                                           DistanceMap *distance_map) const;
};
} // namespace nano_fmm
