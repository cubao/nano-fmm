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

    // dist, t, dot
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
    int N() const { return N_; }
    std::optional<Eigen::Vector3d> k() const { return k_; }
    bool is_wgs84() const { return (bool)k_; }

  private:
    const RowVectors polyline_;
    const int N_;
    const std::optional<Eigen::Vector3d> k_;

    mutable std::optional<std::vector<LineSegment>> segments_;
    mutable std::optional<Eigen::VectorXd> ranges_;

    const std::vector<LineSegment> &segments() const {
        //
        return *segments_; }
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
