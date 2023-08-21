#include <pybind11/eigen.h>
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include "nano_fmm/utils.hpp"

namespace nano_fmm
{
namespace py = pybind11;
using namespace pybind11::literals;
using rvp = py::return_value_policy;

void bind_benchmarks(py::module &m)
{
    m //
        .def(
            "cheap_ruler_k",
            [](int round) {
                Eigen::Vector3d k(0, 0, 0);
                for (int i = 0; i < round; ++i) {
                    for (double l = 0; l < 90.0; l += 0.5) {
                        k += utils::cheap_ruler_k(l);
                    }
                }
                return k;
            },
            "round"_a = 1000)
        .def(
            "cheap_ruler_k_lookup_table",
            [](int round) {
                Eigen::Vector3d k(0, 0, 0);
                for (int i = 0; i < round; ++i) {
                    for (double l = 0; l < 90.0; l += 0.5) {
                        k += utils::cheap_ruler_k_lookup_table(l);
                    }
                }
                return k;
            },
            "round"_a = 1000)
        //
        ;
}
} // namespace nano_fmm
