#pragma once

#include "nano_fmm/types.hpp"
#include <optional>

namespace nano_fmm
{
namespace utils
{

// https://github.com/cubao/headers/blob/main/include/cubao/crs_transform.hpp
inline Eigen::Vector3d cheap_ruler_k(double latitude)
{
    // based on https://github.com/mapbox/cheap-ruler-cpp
    static constexpr double RE = 6378.137;
    static constexpr double FE = 1.0 / 298.257223563;
    static constexpr double E2 = FE * (2 - FE);
    static constexpr double RAD = M_PI / 180.0;
    static constexpr double MUL = RAD * RE * 1000.;
    double coslat = std::cos(latitude * RAD);
    double w2 = 1 / (1 - E2 * (1 - coslat * coslat));
    double w = std::sqrt(w2);
    return Eigen::Vector3d(MUL * w * coslat, MUL * w * w2 * (1 - E2), 1.0);
}

inline RowVectors lla2enu(const Eigen::Ref<const RowVectors> &llas,
                          std::optional<Eigen::Vector3d> anchor_lla = {},
                          std::optional<Eigen::Vector3d> k = {})
{
    if (!llas.rows()) {
        return RowVectors(0, 3);
    }
    if (!anchor_lla) {
        anchor_lla = llas.row(0);
    }
    if (!k) {
        k = cheap_ruler_k((*anchor_lla)[1]);
    }
    RowVectors enus = llas;
    for (int i = 0; i < 3; ++i) {
        enus.col(i).array() -= (*anchor_lla)[i];
        enus.col(i).array() *= (*k)[i];
    }
    return enus;
}
inline RowVectors enu2lla(const Eigen::Ref<const RowVectors> &enus,
                          const Eigen::Vector3d &anchor_lla,
                          std::optional<Eigen::Vector3d> k = {})
{
    if (!enus.rows()) {
        return RowVectors(0, 3);
    }
    auto k = cheap_ruler_k(anchor_lla[1]);
    RowVectors llas = enus;
    for (int i = 0; i < 3; ++i) {
        llas.col(i).array() /= k[i];
        llas.col(i).array() += anchor_lla[i];
    }
    return llas;
}

// https://github.com/cubao/headers/blob/main/include/cubao/eigen_helpers.hpp

inline Eigen::VectorXi
indexes2mask(const Eigen::Ref<const Eigen::VectorXi> &indexes, int N)
{
    Eigen::VectorXi mask(N);
    mask.setZero();
    for (int c = 0, C = indexes.size(); c < C; ++c) {
        mask[indexes[c]] = 1;
    }
    return mask;
}

inline Eigen::VectorXi
mask2indexes(const Eigen::Ref<const Eigen::VectorXi> &mask)
{
    Eigen::VectorXi indexes(mask.sum());
    for (int i = 0, j = 0, N = mask.size(); i < N; ++i) {
        if (mask[i]) {
            indexes[j++] = i;
        }
    }
    return indexes;
}

inline RowVectors to_Nx3(const Eigen::Ref<const RowVectorsNx2> &coords)
{
    RowVectors _coords(coords.rows(), 3);
    _coords.leftCols(2) = coords;
    _coords.col(2).setZero();
    return _coords;
}
} // namespace utils
} // namespace nano_fmm
