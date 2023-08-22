#include "nano_fmm/network.hpp"
#include "nano_fmm/utils.hpp"
#include "spdlog/spdlog.h"

namespace nano_fmm
{

std::shared_ptr<Config> Network::config()
{
    if (!config_) {
        config_ = std::make_shared<Config>();
    }
    return config_;
}

void Network::config(std::shared_ptr<Config> config) { config_ = config; }

void Network::add_road(const Eigen::Ref<RowVectors> &geom, int64_t road_id)
{
    if (roads_.find(road_id) != roads_.end()) {
        spdlog::error("duplicate road, id={}", road_id);
        return;
    }
    if (is_wgs84_) {
        roads_.emplace(
            road_id,
            Polyline(geom, utils::cheap_ruler_k_lookup_table(geom(0, 1))));
    } else {
        roads_.emplace(road_id, Polyline(geom));
    }
    rtree_.reset();
}
void Network::add_link(int64_t source_road, int64_t target_road)
{
    nexts_[source_road].insert(target_road);
    prevs_[target_road].insert(source_road);
}

void Network::remove_road(int64_t road_id)
{
    // TODO
    rtree_.reset();
}
void Network::remove_link(int64_t source_road, int64_t target_road)
{
    // TODO
}
std::unordered_set<int64_t> Network::prev_roads(int64_t road_id) const
{
    auto itr = prevs_.find(road_id);
    if (itr == prevs_.end()) {
        return {};
    }
    return itr->second;
}
std::unordered_set<int64_t> Network::next_roads(int64_t road_id) const
{
    auto itr = nexts_.find(road_id);
    if (itr == nexts_.end()) {
        return {};
    }
    return itr->second;
}
std::unordered_set<int64_t> Network::roads() const
{
    auto ret = std::unordered_set<int64_t>{};
    for (auto &pair : roads_) {
        ret.insert(pair.first);
    }
    return ret;
}

std::vector<ProjectedPoint> Network::query(const Eigen::Vector3d &position,
                                           double radius, std::optional<int> k)
{
    //
    return {};
}

std::unique_ptr<Network> Network::load(const std::string &path)
{
    //
    return {};
}
bool Network::dump(const std::string &path) const
{
    //
    return false;
}

FlatGeobuf::PackedRTree &Network::rtree() const
{
    if (rtree_) {
        return *rtree_;
    }
    segs_.clear();
    seg2idx_.clear();

    using namespace FlatGeobuf;

    auto nodes = std::vector<NodeItem>{};
    for (auto &pair : roads_) {
        int64_t poly_idx = pair.first;
        auto &polyline = pair.second.polyline();
        for (int64_t seg_idx = 0, N = polyline.rows(); seg_idx < N - 1;
             ++seg_idx) {
            auto index = std::make_pair(poly_idx, seg_idx);
            seg2idx_[index] = segs_.size();
            segs_.push_back(index);
            double x0 = polyline(seg_idx, 0);
            double y0 = polyline(seg_idx, 1);
            double x1 = polyline(seg_idx + 1, 0);
            double y1 = polyline(seg_idx + 1, 1);
            if (x0 > x1) {
                std::swap(x0, x1);
            }
            if (y0 > y1) {
                std::swap(y0, y1);
            }
            nodes.push_back({x0, y0, x1, y1, segs_.size()});
        }
    }
    auto extent = calcExtent(nodes);
    hilbertSort(nodes);
    rtree_ = FlatGeobuf::PackedRTree(nodes, extent);
    return *rtree_;
}

} // namespace nano_fmm
