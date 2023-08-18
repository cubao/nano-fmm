#pragma once

#include "nano_fmm/types.hpp"
#include "nano_fmm/polyline.hpp"

#include <optional>
#include <vector>
#include <unordered_map>

namespace nano_fmm
{
struct Network
{
    Network(bool is_wgs84 = true) : is_wgs84_(is_wgs84) {}

    void add(const Eigen::Ref<RowVectors> &polyline, int64_t id = -1);
    void add(const std::vector<RowVectors> &polylines,
             std::optional<int64_t> ids = std::nullopt);

    int load(const std::string &path);
    bool dump(const std::string &path) const;

  private:
    bool is_wgs84_{true};
    std::vector<int64_t> ids_;
    std::unordered_map<int64_t, size_t> id2index_;
    std::unordered_map<int64_t, Polyline> polylines_;
};
} // namespace nano_fmm
