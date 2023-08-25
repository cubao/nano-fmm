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

void bind_utils(py::module &m)
{
    m //
        .def("cheap_ruler_k", &utils::cheap_ruler_k, "latitude"_a)
        .def("cheap_ruler_k_lookup_table", &utils::cheap_ruler_k_lookup_table,
             "latitude"_a)
        .def("bbox",
             py::overload_cast<const Eigen::Vector2d &, double, double>(
                 &utils::bbox),
             "lon_lat"_a, py::kw_only(), "width"_a, "height"_a)
        .def("bbox",
             py::overload_cast<const Eigen::Vector2d &, double>(&utils::bbox),
             "lon_lat"_a, py::kw_only(), "size"_a)
        .def("lla2enu", &utils::lla2enu,    //
             "llas"_a, py::kw_only(),       //
             "anchor_lla"_a = std::nullopt, //
             "k"_a = std::nullopt)
        .def("enu2lla", &utils::enu2lla, //
             "enus"_a, py::kw_only(),    //
             "anchor_lla"_a,             //
             "k"_a = std::nullopt)
        .def("index2mask", &utils::index2mask, "index"_a, "N"_a)
        .def("mask2index", &utils::mask2index, "mask"_a)
        .def("select_by_index", &utils::select_by_index, "coords"_a, "index"_a)
        .def("to_Nx3", &utils::to_Nx3, "coords"_a)
        .def("remove_duplicates", &utils::remove_duplicates, "coords"_a,
             py::kw_only(), "just_xy"_a = true, "is_polyline"_a = true)
        // douglas
        .def("douglas_simplify_mask", &utils::douglas_simplify_mask, "coords"_a,
             py::kw_only(), "epsilon"_a, "is_wgs84"_a)
        .def("douglas_simplify_index", &utils::douglas_simplify_index,
             "coords"_a, py::kw_only(), "epsilon"_a, "is_wgs84"_a)
        .def("douglas_simplify", &utils::douglas_simplify, "coords"_a,
             py::kw_only(), "epsilon"_a, "is_wgs84"_a)
        //
        ;
}
} // namespace nano_fmm
