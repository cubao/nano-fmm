#pragma once

// https://github.com/microsoft/vscode-cpptools/issues/9692
#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__
#endif

#include <Eigen/Core>
#include <Eigen/Dense>
#include <optional>
#include <vector>
#include <rapidjson/document.h>

namespace nano_fmm
{
// Nx3 vectors (row major, just like numpy ndarray)
using RowVectors = Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>;
using RowVectorsNx3 = RowVectors;
// without-z
using RowVectorsNx2 = Eigen::Matrix<double, Eigen::Dynamic, 2, Eigen::RowMajor>;

using VectorUi64 = Eigen::Matrix<uint64_t, Eigen::Dynamic, 1>;
using IndexIJ = Eigen::Matrix<int64_t, 1, 2>;
using IndexIJK = Eigen::Matrix<int64_t, 1, 3>;

// Use the CrtAllocator, because the MemoryPoolAllocator is broken on ARM
// https://github.com/miloyip/rapidjson/issues/200, 301, 388
using RapidjsonAllocator = rapidjson::CrtAllocator;
using RapidjsonDocument =
    rapidjson::GenericDocument<rapidjson::UTF8<>, RapidjsonAllocator>;
using RapidjsonValue =
    rapidjson::GenericValue<rapidjson::UTF8<>, RapidjsonAllocator>;

#ifndef SETUP_FLUENT_API
#define SETUP_FLUENT_API(Klass, VarType, VarName)                              \
    Klass &VarName(const VarType &v)                                           \
    {                                                                          \
        VarName##_ = v;                                                        \
        return *this;                                                          \
    }                                                                          \
    VarType &VarName() { return VarName##_; }                                  \
    const VarType &VarName() const { return VarName##_; }
#endif

// https://github.com/cubao/pybind11-rdp/blob/master/src/main.cpp
struct LineSegment
{
    // LineSegment(A -> B)
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    const Eigen::Vector3d A, B, AB;
    const double len2, inv_len2;
    LineSegment(const Eigen::Vector3d &a, const Eigen::Vector3d &b)
        : A(a), B(b), AB(b - a), //
          len2(AB.squaredNorm()),
          inv_len2(1.0 / (std::numeric_limits<double>::epsilon() + len2))
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
            return std::make_tuple(A, (P - A).norm(), 0.0);
        } else if (dot >= len2) {
            return std::make_tuple(B, (P - B).norm(), 1.0);
        }
        Eigen::Vector3d PP = A + (dot * inv_len2 * AB);
        return std::make_tuple(PP, (PP - P).norm(), dot * inv_len2);
    }
    double t(const Eigen::Vector3d &P) const
    {
        return (P - A).dot(AB) * inv_len2;
    }

    Eigen::Vector3d interpolate(double t) const
    {
        return A * (1.0 - t) + B * t;
    }

    const void build() const
    {
        length();
        dir();
    }

    double length() const
    {
        if (!length_) {
            length_ = std::sqrt(len2);
        }
        return *length_;
    }
    const Eigen::Vector3d &dir() const
    {
        if (!dir_) {
            dir_ = AB * std::sqrt(inv_len2);
        }
        return *dir_;
    }

  private:
    mutable std::optional<Eigen::Vector3d> dir_;
    mutable std::optional<double> length_;
};

// https://github.com/isl-org/Open3D/blob/179886dfd57797b2b0d379062387c60313a58b2b/cpp/open3d/utility/Helper.h#L71
template <typename T> struct hash_eigen
{
    std::size_t operator()(T const &matrix) const
    {
        size_t hash_seed = 0;
        for (int i = 0; i < (int)matrix.size(); i++) {
            auto elem = *(matrix.data() + i);
            hash_seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 +
                         (hash_seed << 6) + (hash_seed >> 2);
        }
        return hash_seed;
    }
};
} // namespace nano_fmm
