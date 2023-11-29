#pragma once

#include "nano_fmm/types.hpp"
#include <optional>

#define _USE_MATH_DEFINES
#include <cmath>

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
    double w2 = 1.0 / (1.0 - E2 * (1.0 - coslat * coslat));
    double w = std::sqrt(w2);
    return Eigen::Vector3d(MUL * w * coslat, MUL * w * w2 * (1 - E2), 1.0);
}

inline Eigen::Vector3d cheap_ruler_k_lookup_table(double latitude)
{
    // lookup table
#ifdef K
#undef K
#endif
#define K(lat) utils::cheap_ruler_k((double)lat)
    const static Eigen::Vector3d Ks[] = {
        // clang-format off
        K(0),K(1),K(2),K(3),K(4),K(5),K(6),K(7),K(8),K(9),K(10),K(11),K(12),K(13),
        K(14),K(15),K(16),K(17),K(18),K(19),K(20),K(21),K(22),K(23),K(24),K(25),K(26),
        K(27),K(28),K(29),K(30),K(31),K(32),K(33),K(34),K(35),K(36),K(37),K(38),K(39),
        K(40),K(41),K(42),K(43),K(44),K(45),K(46),K(47),K(48),K(49),K(50),K(51),K(52),
        K(53),K(54),K(55),K(56),K(57),K(58),K(59),K(60),K(61),K(62),K(63),K(64),K(65),
        K(66),K(67),K(68),K(69),K(70),K(71),K(72),K(73),K(74),K(75),K(76),K(77),K(78),
        K(79),K(80),K(81),K(82),K(83),K(84),K(85),K(86),K(87),K(88),K(89),K(90)
        // clang-format on
    };
#undef K
    int idx =
        std::min(90, static_cast<int>(std::floor(std::fabs(latitude) + 0.5)));
    return Ks[idx];
}

inline Eigen::Vector3d offset(const Eigen::Vector3d &lla_src,
                              const Eigen::Vector3d &lla_dst)
{
    return (lla_dst - lla_src).array() *
           cheap_ruler_k_lookup_table(lla_src[1]).array();
}

inline Eigen::Vector4d bbox(const Eigen::Vector2d &lon_lat, //
                            double width, double height)
{
    auto k = cheap_ruler_k_lookup_table(lon_lat[1]);
    double dlon = 1.0 / k[0] * width / 2.0;
    double dlat = 1.0 / k[1] * height / 2.0;
    return Eigen::Vector4d(lon_lat[0] - dlon, lon_lat[1] - dlat,
                           lon_lat[0] + dlon, lon_lat[1] + dlat);
}
inline Eigen::Vector4d bbox(const Eigen::Vector2d &lon_lat, double size)
{
    return bbox(lon_lat, size, size);
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
        k = cheap_ruler_k_lookup_table((*anchor_lla)[1]);
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
    if (!k) {
        k = cheap_ruler_k_lookup_table(anchor_lla[1]);
    }
    RowVectors llas = enus;
    for (int i = 0; i < 3; ++i) {
        llas.col(i).array() /= (*k)[i];
        llas.col(i).array() += anchor_lla[i];
    }
    return llas;
}

// https://github.com/cubao/headers/blob/main/include/cubao/eigen_helpers.hpp

inline Eigen::VectorXi
index2mask(const Eigen::Ref<const Eigen::VectorXi> &indexes, int N)
{
    Eigen::VectorXi mask(N);
    mask.setZero();
    for (int c = 0, C = indexes.size(); c < C; ++c) {
        mask[indexes[c]] = 1;
    }
    return mask;
}

inline Eigen::VectorXi mask2index(const Eigen::Ref<const Eigen::VectorXi> &mask)
{
    Eigen::VectorXi indexes(mask.sum());
    for (int i = 0, j = 0, N = mask.size(); i < N; ++i) {
        if (mask[i]) {
            indexes[j++] = i;
        }
    }
    return indexes;
}

inline RowVectors select_by_index(const Eigen::Ref<const RowVectors> &coords,
                                  const Eigen::VectorXi &index)
{
    RowVectors selected(index.size(), 3);
    int j = -1;
    for (int i = 0; i < index.size(); ++i) {
        selected.row(++j) = coords.row(index[i]);
    }
    return selected;
}

inline RowVectors to_Nx3(const Eigen::Ref<const RowVectorsNx2> &coords)
{
    RowVectors _coords(coords.rows(), 3);
    _coords.leftCols(2) = coords;
    _coords.col(2).setZero();
    return _coords;
}

inline RowVectors remove_duplicates(const Eigen::Ref<const RowVectors> &coords,
                                    bool just_xy = true,
                                    bool is_polyline = true)
{
    const int N = coords.rows();
    std::vector<int> selected;
    selected.reserve(N);
    selected.push_back(0);
    if (just_xy) {
        for (int i = 1; i < N; ++i) {
            if (coords(i, 0) == coords(selected.back(), 0) &&
                coords(i, 1) == coords(selected.back(), 1)) {
                continue;
            }
            selected.push_back(i);
        }
    } else {
        for (int i = 1; i < N; ++i) {
            if (coords(i, 0) == coords(selected.back(), 0) &&
                coords(i, 1) == coords(selected.back(), 1) &&
                coords(i, 2) == coords(selected.back(), 2)) {
                continue;
            }
            selected.push_back(i);
        }
    }
    if (is_polyline && selected.size() == 1) {
        selected.push_back(N - 1);
    }
    return select_by_index(coords,
                           Eigen::VectorXi::Map(&selected[0], selected.size()));
}

// https://github.com/cubao/pybind11-rdp/blob/master/src/main.cpp
inline void __douglas_simplify(const Eigen::Ref<const RowVectors> &coords,
                               Eigen::VectorXi &to_keep, const int i,
                               const int j, const double epsilon)
{
    to_keep[i] = to_keep[j] = 1;
    if (j - i <= 1) {
        return;
    }
    LineSegment line(coords.row(i), coords.row(j));
    double max_dist2 = 0.0;
    int max_index = i;
    int mid = i + (j - i) / 2;
    int min_pos_to_mid = j - i;
    for (int k = i + 1; k < j; ++k) {
        double dist2 = line.distance2(coords.row(k));
        if (dist2 > max_dist2) {
            max_dist2 = dist2;
            max_index = k;
        } else if (dist2 == max_dist2) {
            // a workaround to ensure we choose a pivot close to the middle of
            // the list, reducing recursion depth, for certain degenerate inputs
            // https://github.com/mapbox/geojson-vt/issues/104
            int pos_to_mid = std::fabs(k - mid);
            if (pos_to_mid < min_pos_to_mid) {
                min_pos_to_mid = pos_to_mid;
                max_index = k;
            }
        }
    }
    if (max_dist2 <= epsilon * epsilon) {
        return;
    }
    __douglas_simplify(coords, to_keep, i, max_index, epsilon);
    __douglas_simplify(coords, to_keep, max_index, j, epsilon);
}

inline Eigen::VectorXi
douglas_simplify_mask(const Eigen::Ref<const RowVectors> &coords,
                      double epsilon, bool is_wgs84)
{
    Eigen::VectorXi mask(coords.rows());
    mask.setZero();
    if (is_wgs84) {
        auto enus =
            lla2enu(coords, {}, cheap_ruler_k_lookup_table(coords(0, 1)));
        __douglas_simplify(enus, mask, 0, mask.size() - 1, epsilon);
    } else {
        __douglas_simplify(coords, mask, 0, mask.size() - 1, epsilon);
    }
    return mask;
}

inline Eigen::VectorXi
douglas_simplify_index(const Eigen::Ref<const RowVectors> &coords,
                       double epsilon, bool is_wgs84)
{
    return mask2index(douglas_simplify_mask(coords, epsilon, is_wgs84));
}

inline RowVectors douglas_simplify(const Eigen::Ref<const RowVectors> &coords,
                                   double epsilon, bool is_wgs84)
{
    return select_by_index(coords,
                           douglas_simplify_index(coords, epsilon, is_wgs84));
}

} // namespace utils
} // namespace nano_fmm
