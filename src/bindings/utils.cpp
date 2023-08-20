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
        .def("to_Nx3", &utils::to_Nx3, "coords"_a)
        //
        ;
}
} // namespace nano_fmm
