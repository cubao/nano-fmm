#include "nano_fmm/network.hpp"
#include "nano_fmm/utils.hpp"
#include "nano_fmm/heap.hpp"
#include "spdlog/spdlog.h"

#include <execution>

namespace nano_fmm
{
bool Network::add_road(const Eigen::Ref<RowVectors> &geom, int64_t road_id)
{
    if (roads_.find(road_id) != roads_.end()) {
        spdlog::error("duplicate road, id={}, should remove_road first",
                      road_id);
        return false;
    }
    if (is_wgs84_) {
        roads_.emplace(
            road_id,
            Polyline(geom, utils::cheap_ruler_k_lookup_table(geom(0, 1))));
    } else {
        roads_.emplace(road_id, Polyline(geom));
    }
    rtree_.reset();
    return true;
}
bool Network::add_link(int64_t source_road, int64_t target_road)
{
    if (roads_.find(source_road) == roads_.end()) {
        spdlog::error("source_road={} not in network", source_road);
        return false;
    }
    if (roads_.find(target_road) == roads_.end()) {
        spdlog::error("target_road={} not in network", target_road);
        return false;
    }
    nexts_[source_road].insert(target_road);
    prevs_[target_road].insert(source_road);
    return true;
}

bool Network::remove_road(int64_t road_id)
{
    if (roads_.erase(road_id)) {
        rtree_.reset();
        return true;
    }
    return false;
}
bool Network::remove_link(int64_t source_road, int64_t target_road)
{
    auto itr = nexts_.find(source_road);
    if (itr == nexts_.end()) {
        return false;
    }
    if (itr->second.erase(target_road)) {
        prevs_[target_road].erase(source_road);
        return true;
    }
    return false;
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

const Polyline *Network::road(int64_t road_id) const
{
    auto itr = roads_.find(road_id);
    if (itr == roads_.end()) {
        return nullptr;
    }
    return &itr->second;
}

std::vector<ProjectedPoint>
Network::query(const Eigen::Vector3d &position, double radius,
               std::optional<int> k, std::optional<double> z_max_offset) const
{
    double x = position[0], y = position[1];
    double dx = radius, dy = radius;
    if (is_wgs84_) {
        auto kk = utils::cheap_ruler_k_lookup_table(position[1]);
        dx /= kk[0];
        dy /= kk[1];
    }
    auto &tree = this->rtree();
    auto hits = tree.search(x - dx, y - dy, x + dx, y + dy);
    auto poly2seg_minmax =
        std::unordered_map<int64_t, std::pair<int64_t, int64_t>>();
    for (auto &hit : hits) {
        auto poly_seg = segs_[hit.offset];
        auto poly_idx = poly_seg[0];
        auto seg_idx = poly_seg[1];
        auto itr = poly2seg_minmax.find(poly_idx);
        if (itr == poly2seg_minmax.end()) {
            poly2seg_minmax.emplace(poly_idx, std::make_pair(seg_idx, seg_idx));
        } else {
            if (seg_idx < itr->second.first) {
                itr->second.first = seg_idx;
            }
            if (seg_idx > itr->second.second) {
                itr->second.second = seg_idx;
            }
        }
    }
    auto nearests = std::vector<ProjectedPoint>();
    nearests.reserve(poly2seg_minmax.size());
    for (auto &pair : poly2seg_minmax) {
        auto &poly = roads_.at(pair.first);
        auto [P, d, s, t] =
            poly.nearest(position, pair.second.first, pair.second.second);
        if (z_max_offset && std::fabs(P[2] - position[2]) > *z_max_offset) {
            continue;
        }
        if (d > radius) {
            continue;
        }
        nearests.emplace_back(P, poly.segment(s).dir(), d, //
                              pair.first, poly.range(s, t));
    }
    std::sort(nearests.begin(), nearests.end(),
              [](auto &n1, auto &n2) { return n1.distance() < n2.distance(); });
    if (k && nearests.size() > *k) {
        nearests.resize(*k);
    }
    return nearests;
}

std::map<std::tuple<int64_t, int64_t>, RowVectors>
Network::query(const Eigen::Vector4d &bbox) const
{
    auto ret = std::map<std::tuple<int64_t, int64_t>, RowVectors>();
    auto &tree = this->rtree();
    auto hits = tree.search(bbox[0], bbox[1], bbox[2], bbox[3]);
    for (auto &hit : hits) {
        auto poly_seg = segs_[hit.offset];
        auto poly_idx = poly_seg[0];
        auto seg_idx = poly_seg[1];
        ret.emplace(std::make_tuple(poly_idx, seg_idx),
                    roads_.at(poly_idx).polyline().middleRows(seg_idx, 2));
    }
    return ret;
}

MatchResult Network::match(const RowVectors &trajectory) const
{
    // MatchResult FastMapMatch::match_traj(const Trajectory &traj, const
    // FastMapMatchConfig &config)

    // Traj_Candidates tc = network_.search_tr_cs_knn(traj.geom, config.k,
    // config.radius); if (tc.empty())
    //     return MatchResult{};

    // TransitionGraph tg(tc, config.gps_error);
    // update_tg(&tg, traj, config.reverse_tolerance);
    // TGOpath tg_opath = tg.backtrack();
    // SPDLOG_DEBUG("Optimal path size {}", tg_opath.size());

    // MatchedCandidatePath matched_candidate_path(tg_opath.size());
    // std::transform(
    //     tg_opath.begin(), tg_opath.end(), matched_candidate_path.begin(),
    //     [](const TGNode *a) {
    //         return MatchedCandidate{*(a->c), a->ep, a->tp, a->sp_dist};
    //     });

    // O_Path opath(tg_opath.size());
    // std::transform(tg_opath.begin(), tg_opath.end(), opath.begin(),
    //                [](const TGNode *a) { return a->c->edge->id; });

    // std::vector<int> indices;
    // const std::vector<Edge> &edges = network_.get_edges();
    // C_Path cpath = ubodt_->construct_complete_path(
    //     traj.id, tg_opath, edges, &indices, config.reverse_tolerance);

    // LineString mgeom = network_.complete_path_to_geometry(traj.geom, cpath);

    // return MatchResult{traj.id, matched_candidate_path, opath, cpath,
    // indices,
    //                    mgeom};
    MatchResult ret;
    return ret;
}

void Network::build(int execution_polylicy) const
{
    if (execution_polylicy == 1) {
        std::for_each(std::execution::par, roads_.begin(), roads_.end(),
                      [](auto &pair) { pair.second.build(); });
    } else if (execution_polylicy == 2) {
        std::for_each(std::execution::par_unseq, roads_.begin(), roads_.end(),
                      [](auto &pair) { pair.second.build(); });
    } else {
        std::for_each(std::execution::seq, roads_.begin(), roads_.end(),
                      [](auto &pair) { pair.second.build(); });
    }
    rtree();
}

void Network::reset() const { rtree_.reset(); }

std::unique_ptr<Network> Network::load(const std::string &path)
{
    //
    return {};
}
bool Network::dump(const std::string &path, bool with_config) const
{
    //
    return false;
}

std::vector<UbodtRecord>
Network::build_ubodt(std::optional<double> thresh) const
{
    auto roads = std::vector<int64_t>{};
    roads.reserve(roads_.size());
    for (auto &pair : roads_) {
        roads.push_back(pair.first);
    }
    return build_ubodt(roads, thresh);
}

std::vector<UbodtRecord>
Network::build_ubodt(const std::vector<int64_t> &roads,
                     std::optional<double> thresh) const
{
    if (!thresh) {
        thresh = config_.ubodt_thresh;
    }
    auto records = std::vector<UbodtRecord>();
    for (auto s : roads) {
        IndexMap pmap;
        DistanceMap dmap;
        single_source_upperbound_dijkstra(s, *thresh, pmap, dmap);
        for (const auto &iter : pmap) {
            auto curr = iter.first;
            if (curr == s) {
                continue;
            }
            const auto prev = iter.second;
            auto succ = curr;
            int64_t u;
            while ((u = pmap[succ]) != s) {
                succ = u;
            }
            records.push_back({s, curr, succ, prev, dmap[curr]});
        }
    }
    return records;
}

size_t Network::clear_ubodt()
{
    size_t count = ubodt_.size();
    ubodt_.clear();
    return count;
}

size_t Network::load_ubodt(const std::vector<UbodtRecord> &rows)
{
    // ubodt_;
    size_t count = 0;
    for (auto &row : rows) {
        if (ubodt_.emplace(IndexIJ(row.source_road(), row.target_road()), row)
                .second) {
            ++count;
        }
    }
    return count;
}

bool Network::load_ubodt(const std::string &path)
{
    //
    return false;
}
bool Network::dump_ubodt(const std::string &path,
                         std::optional<double> thresh) const
{
    //
    return false;
}

Network Network::to_2d() const
{
    auto net = Network(is_wgs84_);
    net.config(config_);
    for (auto &pair : roads_) {
        auto xy0 = utils::to_Nx3(pair.second.polyline().leftCols(2));
        net.add_road(xy0, pair.first);
    }
    for (auto &pair : nexts_) {
        auto curr = pair.first;
        for (auto next : pair.second) {
            net.add_link(curr, next);
        }
    }
    return net;
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
    uint64_t ii = 0;
    for (auto &pair : roads_) {
        int64_t poly_idx = pair.first;
        auto &polyline = pair.second.polyline();
        for (int64_t seg_idx = 0, N = polyline.rows(); seg_idx < N - 1;
             ++seg_idx) {
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
            nodes.push_back({x0, y0, x1, y1, ii});
            IndexIJ index(poly_idx, seg_idx);
            seg2idx_[index] = ii;
            segs_.push_back(index);
            ++ii;
        }
    }
    auto extent = calcExtent(nodes);
    hilbertSort(nodes);
    rtree_ = FlatGeobuf::PackedRTree(nodes, extent);
    return *rtree_;
}

void Network::single_source_upperbound_dijkstra(int64_t s, double delta, //
                                                IndexMap &pmap,
                                                DistanceMap &dmap) const
{
    Heap Q;
    Q.push(s, -roads_.at(s).length());
    pmap.insert({s, s});
    dmap.insert({s, 0});
    while (!Q.empty()) {
        HeapNode node = Q.top();
        Q.pop();
        if (node.value > delta)
            break;
        auto u = node.index;
        auto itr = nexts_.find(u);
        if (itr == nexts_.end()) {
            continue;
        }
        double u_cost = roads_.at(u).length();
        for (auto v : itr->second) {
            auto c = node.value + u_cost;
            auto iter = dmap.find(v);
            if (iter != dmap.end()) {
                if (c < iter->second) {
                    pmap[v] = u;
                    dmap[v] = c;
                    Q.decrease_key(v, c);
                };
            } else {
                if (c <= delta) {
                    Q.push(v, c);
                    pmap.insert({v, u});
                    dmap.insert({v, c});
                }
            }
        }
    }
}

} // namespace nano_fmm
