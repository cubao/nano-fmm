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

struct Network
{
    Network(bool is_wgs84 = false) : is_wgs84_(is_wgs84) {}

    // road network
    bool add_road(const Eigen::Ref<RowVectors> &geom, int64_t road_id);
    bool add_link(int64_t source_road, int64_t target_road);
    bool remove_road(int64_t road_id);
    bool remove_link(int64_t source_road, int64_t target_road);
    std::unordered_set<int64_t> prev_roads(int64_t road_id) const;
    std::unordered_set<int64_t> next_roads(int64_t road_id) const;
    std::unordered_set<int64_t> roads() const;
    const Polyline *road(int64_t road_id) const;

    // config
    const Config &config() const { return config_; }
    Config &config() { return config_; }
    Network &config(const Config &new_config)
    {
        config_ = new_config;
        return *this;
    }

    // query
    std::vector<ProjectedPoint>
    query(const Eigen::Vector3d &position, double radius,
          std::optional<int> k = std::nullopt,
          std::optional<double> z_max_offset = std::nullopt) const;
    std::map<std::tuple<int64_t, int64_t>, RowVectors>
    query(const Eigen::Vector4d &bbox) const;

    // build cache (not necessary)
    void build() const;

    // graph operations
    // move forward/backward N meters, return ProjectPoint
    // single source dijkstra

    // load&dump
    static std::unique_ptr<Network> load(const std::string &path);
    bool dump(const std::string &path, bool with_config = true) const;
    // ubodt
    std::vector<UbodtRecord>
    build_ubodt(std::optional<double> thresh = std::nullopt) const;
    std::vector<UbodtRecord>
    build_ubodt(const std::vector<int64_t> &roads,
                std::optional<double> thresh = std::nullopt) const;
    size_t clear_ubodt();
    size_t load_ubodt(const std::vector<UbodtRecord> &rows);
    bool load_ubodt(const std::string &path);
    bool dump_ubodt(const std::string &path,
                    std::optional<double> thresh) const;

    // to 2d, z will be set to zero
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
                                           IndexMap &predecessor_map,
                                           DistanceMap &distance_map) const;
    std::unordered_map<IndexIJ, UbodtRecord> ubodt_;
};
} // namespace nano_fmm
