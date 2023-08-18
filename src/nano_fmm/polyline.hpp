#pragma once

#include "nano_fmm/types.hpp"
#include <optional>

namespace nano_fmm
{
// https://github.com/cubao/pybind11-rdp/blob/master/src/main.cpp
struct LineSegment
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    const Eigen::Vector3d A, B, AB;
    const double len2, inv_len2;
    LineSegment(const Eigen::Vector3d &a, const Eigen::Vector3d &b)
        : A(a), B(b), AB(b - a), //
          len2((b - a).squaredNorm()), inv_len2(1.0 / len2)
    {
    }
    double distance2(const Eigen::Vector3d &P) const
    {
        double dot = (P - A).dot(AB);
        if (dot <= 0) {
            return (P - A).squaredNorm();
        } else if (dot >= len2) {
            return (P - B).squaredNorm();
        }
        // P' = A + dot/length * normed(AB)
        //    = A + dot * AB / (length^2)
        return (A + (dot * inv_len2 * AB) - P).squaredNorm();
    }
    double distance(const Eigen::Vector3d &P) const
    {
        return std::sqrt(distance2(P));
    }
    Eigen::Vector3d interpolate(double t) const
    {
        // 0 -> A, 1 -> B
        return A;
    }

    // dist, t, dot
};

// https://github.com/cubao/headers/blob/main/include/cubao/polyline_ruler.hpp
struct Polyline
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Polyline(const Eigen::Ref<const RowVectors> &polyline,
             const std::optional<Eigen::Vector3d> scale = {})
        : polyline_(polyline), //
          N_(polyline.rows()), //
          scale_(scale)
    {
    }

    const RowVectors &polyline() const { return polyline_; }
    std::optional<Eigen::Vector3d> scale() const { return scale_; }
    bool is_wgs84() const { return (bool)scale_; }

    double range(int seg_idx) const { return ranges()[seg_idx]; }
    double range(int seg_idx, double t) const
    {
        auto &ranges = this->ranges();
        return ranges[seg_idx] * (1.0 - t) + ranges[seg_idx + 1] * t;
    }

    int segment_index(double range) const
    {
        const double *ranges = this->ranges().data();
        int I = std::upper_bound(ranges, ranges + N_, range) - ranges;
        return std::min(std::max(0, I - 1), N_ - 2);
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

    std::tuple<Eigen::Vector3d, int, double>
    snap(const Eigen::Vector3d &point) const
    {
        return std::make_tuple(Eigen::Vector3d(), 0, 0);
    }
    RowVectors slice(std::optional<double> min, std::optional<double> max) const
    {
        return RowVectors(0, 3);
    }

    static Eigen::Vector3d interpolate(const Eigen::Vector3d &a,
                                       const Eigen::Vector3d &b, double t)
    {
        return a + (b - a) * t;
    }

  private:
    const RowVectors polyline_;
    const int N_;
    const std::optional<Eigen::Vector3d> scale_;

    mutable std::optional<std::vector<LineSegment>> segments_;
    mutable std::optional<Eigen::VectorXd> ranges_;

    const std::vector<LineSegment> &segments() const
    {
        //
        return *segments_;
    }
    const Eigen::VectorXd &ranges() const
    {
        if (ranges_) {
            return *ranges_;
        }
        Eigen::VectorXd ranges(N_);
        int idx = 0;
        for (auto &seg : segments()) {
            ranges[idx++] = std::sqrt(seg.len2);
        }
        ranges_ = std::move(ranges);
        return *ranges_;
    }
};

} // namespace nano_fmm
