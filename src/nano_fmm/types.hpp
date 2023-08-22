#pragma once

// https://github.com/microsoft/vscode-cpptools/issues/9692
#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__
#endif

#include <Eigen/Core>
#include <Eigen/Dense>

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
