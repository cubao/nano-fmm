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
}
void Network::add_link(int64_t source_road, int64_t target_road)
{
    nexts_[source_road].insert(target_road);
    prevs_[target_road].insert(source_road);
}

void Network::remove_road(int64_t road_id)
{
    // TODO
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

Network Network::load(const std::string &path)
{
    //
    return Network();
}
bool Network::dump(const std::string &path) const
{
    //
    return false;
}

} // namespace nano_fmm
