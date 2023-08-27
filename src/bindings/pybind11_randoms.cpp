#include <pybind11/eigen.h>
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include "nano_fmm/randoms.hpp"

namespace nano_fmm
{
namespace py = pybind11;
using namespace pybind11::literals;
using rvp = py::return_value_policy;

void bind_randoms(py::module &m)
{
    m //
        .def("hsv2rgb", &hsv_to_rgb, "h"_a, "s"_a, "v"_a)
        //
        ;
    py::class_<RandomStroke>(m, "RandomStroke", py::module_local()) //
        .def(py::init<int, bool>(), "seed"_a, py::kw_only(),
             "on_black"_a = true)
        .def(py::init<bool>(), py::kw_only(), "on_black"_a = true)
        .def("next", &RandomStroke::next)
        //
        ;
}
} // namespace nano_fmm
