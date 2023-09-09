#pragma once

#include "nano_fmm/types.hpp"
#include "nano_fmm/config.hpp"
#include "nano_fmm/polyline.hpp"

#include "nano_fmm/network/projected_point.hpp"
#include "nano_fmm/network/ubodt.hpp"
#include "nano_fmm/network/match_result.hpp"

#include "packedrtree.hpp"

#include <optional>
#include <memory>
#include <vector>
#include <map>
#include <unordered_map>
#include <unordered_set>

namespace nano_fmm
{
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

    // traj
    MatchResult match(const RowVectors &trajectory) const;

    // build cache (not necessary), 0 -> seq, 1 -> par, 2 -> par_unseq
    void build(int execution_polylicy = 2) const;
    void reset() const;

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

    Network &from_geojson(const RapidjsonValue &json);
    RapidjsonValue to_geojson(RapidjsonAllocator &allocator) const;
    RapidjsonValue to_geojson() const
    {
        RapidjsonAllocator allocator;
        return to_geojson(allocator);
    }

    Network &from_rapidjson(const RapidjsonValue &json);
    RapidjsonValue to_rapidjson(RapidjsonAllocator &allocator) const;
    RapidjsonValue to_rapidjson() const
    {
        RapidjsonAllocator allocator;
        return to_rapidjson(allocator);
    }

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
    std::unordered_map<IndexIJ, UbodtRecord, hash_eigen<IndexIJ>> ubodt_;
};
} // namespace nano_fmm
