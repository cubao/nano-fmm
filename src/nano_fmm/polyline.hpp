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
          len2(AB.squaredNorm()), inv_len2(1.0 / len2)
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
    // return P', distance, t
    std::tuple<Eigen::Vector3d, double, double>
    nearest(const Eigen::Vector3d &P) const
    {
        double dot = (P - A).dot(AB);
        if (dot <= 0) {
            return std::make_tuple(A, (P - A).squaredNorm(), 0.0);
        } else if (dot >= len2) {
            return std::make_tuple(B, (P - B).squaredNorm(), 1.0);
        }
        Eigen::Vector3d PP = A + (dot * inv_len2 * AB);
        return std::make_tuple(PP, (PP - P).squaredNorm(), dot * inv_len2);
    }
    double t(const Eigen::Vector3d &P) const
    {
        return (P - A).dot(AB) * inv_len2;
    }

    Eigen::Vector3d interpolate(double t) const
    {
        return A * (1.0 - t) + B * t;
    }

    double length() const { return std::sqrt(len2); }
    Eigen::Vector3d dir() const { return AB / length(); }
};

// https://github.com/cubao/headers/blob/main/include/cubao/polyline_ruler.hpp
struct Polyline
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Polyline(const Eigen::Ref<const RowVectors> &polyline,
             const std::optional<Eigen::Vector3d> k = {})
        : polyline_(polyline), //
          N_(polyline.rows()), //
          k_(k)
    {
    }

    const RowVectors &polyline() const { return polyline_; }
    std::optional<Eigen::Vector3d> k() const { return k_; }
    bool is_wgs84() const { return (bool)k_; }

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
        if (!k_) {
            for (int i = 1; i < N_; ++i) {
                segments_->emplace_back(polyline_.row(i - 1), polyline_.row(i));
            }
        } else {
            for (int i = 1; i < N_; ++i) {
                Eigen::Vector3d A = polyline_.row(i - 1) - polyline_.row(0);
                Eigen::Vector3d B = polyline_.row(i) - polyline_.row(0);
                A.array() *= k_->array();
                B.array() *= k_->array();
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
        ranges.setZero();
        int idx = 0;
        for (auto &seg : segments()) {
            ranges[idx + 1] = ranges[idx] + seg.length();
            ++idx;
        }
        ranges_ = std::move(ranges);
        return *ranges_;
    }

  private:
    const RowVectors polyline_;
    const int N_;
    const std::optional<Eigen::Vector3d> k_;

    mutable std::optional<std::vector<LineSegment>> segments_;
    mutable std::optional<Eigen::VectorXd> ranges_;
};

} // namespace nano_fmm
