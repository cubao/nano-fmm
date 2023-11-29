#pragma once

#include "nano_fmm/types.hpp"
#include <optional>

#define _USE_MATH_DEFINES
#include <cmath>

namespace nano_fmm
{
// https://github.com/cubao/headers/blob/main/include/cubao/polyline_ruler.hpp
struct Polyline
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Polyline(const Eigen::Ref<const RowVectors> &polyline,
             bool is_wgs84 = false)
        : polyline_(polyline), //
          N_(polyline.rows()), //
          is_wgs84_(is_wgs84),
          k_(is_wgs84 ? cheap_ruler_k(polyline(0, 1)) : Eigen::Vector3d::Ones())
    {
    }
    Polyline(const Eigen::Ref<const RowVectors> &polyline,
             const Eigen::Vector3d &k)
        : polyline_(polyline), //
          N_(polyline.rows()), //
          is_wgs84_(true), k_(k)
    {
    }

    const RowVectors &polyline() const { return polyline_; }
    int N() const { return N_; }
    Eigen::Vector3d k() const { return k_; }
    bool is_wgs84() const { return is_wgs84_; }

    double range(int seg_idx, double t = 0.0) const
    {
        auto &ranges = this->ranges();
        return ranges[seg_idx] * (1.0 - t) + ranges[seg_idx + 1] * t;
    }

    std::pair<int, double> segment_index_t(double range) const
    {
        const double *ranges = this->ranges().data();
        int I = std::upper_bound(ranges, ranges + N_, range) - ranges;
        int i = std::min(std::max(0, I - 1), N_ - 2);
        double t = (range - ranges[i]) / (ranges[i + 1] - ranges[i]);
        return {i, t};
    }
    double length() const { return ranges()[N_ - 1]; }

    Eigen::Vector3d along(double range, bool extend = false) const
    {
        if (!extend) {
            range = std::max(0.0, std::min(range, length()));
        }
        auto [i, t] = segment_index_t(range);
        return interpolate(polyline_.row(i), polyline_.row(i + 1), t);
    }

    // P', distance, seg_idx, t
    std::tuple<Eigen::Vector3d, double, int, double>
    nearest(const Eigen::Vector3d &point,
            std::optional<int> seg_min = std::nullopt,
            std::optional<int> seg_max = std::nullopt) const
    {
        if (!seg_min) {
            seg_min = 0;
        }
        if (!seg_max) {
            seg_max = N_ - 2;
        }
        assert(0 <= *seg_min && *seg_min <= *seg_max && *seg_max <= N_ - 2);
        auto &segs = segments();
        Eigen::Vector3d xyz = point;
        if (is_wgs84_) {
            xyz -= polyline_.row(0);
            xyz.array() *= k_.array();
        }
        Eigen::Vector3d PP = xyz;
        double dd = std::numeric_limits<double>::max();
        int ss = -1;
        double tt = 0.0;
        for (int s = *seg_min; s <= *seg_max; ++s) {
            auto [P, d, t] = segs[s].nearest(xyz);
            if (d < dd) {
                PP = P;
                dd = d;
                ss = s;
                tt = t;
            }
        }
        if (is_wgs84_) {
            PP.array() /= k_.array();
            PP += polyline_.row(0);
        }
        return std::make_tuple(PP, dd, ss, tt);
    }

    RowVectors slice(std::optional<double> min, std::optional<double> max) const
    {
        if (!min) {
            min = 0.0;
        }
        if (!max) {
            max = length();
        }
        if (*min > *max) {
            return RowVectors(0, 3);
        }
        auto [seg0, t0] = segment_index_t(*min);
        auto [seg1, t1] = segment_index_t(*max);
        auto coords = std::vector<Eigen::Vector3d>();
        coords.push_back(
            interpolate(polyline_.row(seg0), polyline_.row(seg0 + 1), t0));
        for (int s = seg0 + 1; s <= seg1; ++s) {
            coords.push_back(polyline_.row(s));
        }
        if (t1 > 0) {
            coords.push_back(
                interpolate(polyline_.row(seg1), polyline_.row(seg1 + 1), t1));
        }
        return Eigen::Map<const RowVectors>(coords[0].data(), coords.size(), 3);
    }

    static Eigen::Vector3d interpolate(const Eigen::Vector3d &a,
                                       const Eigen::Vector3d &b, double t)
    {
        return a + (b - a) * t;
    }

    void build() const
    {
        for (auto &seg : segments()) {
            seg.build();
        }
        ranges();
    }

    const LineSegment &segment(int index) const
    {
        index = index < 0 ? index + N_ - 1 : index;
        return segments()[index];
    }
    const std::vector<LineSegment> &segments() const
    {
        if (segments_) {
            return *segments_;
        }
        segments_ = std::vector<LineSegment>{};
        segments_->reserve(N_ - 1);
        if (!is_wgs84_) {
            for (int i = 1; i < N_; ++i) {
                segments_->emplace_back(polyline_.row(i - 1), polyline_.row(i));
            }
        } else {
            for (int i = 1; i < N_; ++i) {
                Eigen::Vector3d A = polyline_.row(i - 1) - polyline_.row(0);
                Eigen::Vector3d B = polyline_.row(i) - polyline_.row(0);
                A.array() *= k_.array();
                B.array() *= k_.array();
                segments_->emplace_back(A, B);
            }
        }
        return *segments_;
    }
    const Eigen::VectorXd &ranges() const
    {
        if (ranges_) {
            return *ranges_;
        }
        Eigen::VectorXd ranges(N_);
        ranges[0] = 0.0;
        int idx = 0;
        for (auto &seg : segments()) {
            ranges[idx + 1] = ranges[idx] + seg.length();
            ++idx;
        }
        ranges_ = std::move(ranges);
        return *ranges_;
    }

    const Polyline remove_backwards() const
    {
        std::vector<int> selected;
        selected.reserve(N_);
        selected.push_back(0);
        auto &segs = segments();
        for (int i = 1; i < N_ - 1; ++i) {
            if (segs[i].dir().dot(segs[selected.back()].dir()) > 0) {
                selected.push_back(i);
            }
        }
        // just like utils.hpp/select_by_index
        RowVectors coords(selected.size(), 3);
        int j = -1;
        for (auto idx : selected) {
            coords.row(++j) = polyline_.row(idx);
        }
        return is_wgs84_ ? Polyline(coords, k_) : Polyline(coords);
    }

  private:
    const RowVectors polyline_;
    const int N_;
    const bool is_wgs84_;
    const Eigen::Vector3d k_;

    mutable std::optional<std::vector<LineSegment>> segments_;
    mutable std::optional<Eigen::VectorXd> ranges_;

    // same as utils.hpp/cheap_ruler_k
    inline Eigen::Vector3d cheap_ruler_k(double latitude)
    {
        static constexpr double RE = 6378.137;
        static constexpr double FE = 1.0 / 298.257223563;
        static constexpr double E2 = FE * (2 - FE);
        static constexpr double RAD = M_PI / 180.0;
        static constexpr double MUL = RAD * RE * 1000.;
        double coslat = std::cos(latitude * RAD);
        double w2 = 1.0 / (1.0 - E2 * (1.0 - coslat * coslat));
        double w = std::sqrt(w2);
        return Eigen::Vector3d(MUL * w * coslat, MUL * w * w2 * (1 - E2), 1.0);
    }
};

} // namespace nano_fmm
