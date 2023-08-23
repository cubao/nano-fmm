#include <pybind11/eigen.h>
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include "nano_fmm/network.hpp"

namespace nano_fmm
{
namespace py = pybind11;
using namespace pybind11::literals;
using rvp = py::return_value_policy;

void bind_network(py::module &m)
{
    py::class_<Network>(m, "Network", py::module_local()) //
                                                          //
        .def(py::init<bool>(), "is_wgs84"_a = false)
        //
        .def("query", &Network::query, "position"_a, py::kw_only(), "radius"_a,
             "k"_a = std::nullopt);
}
} // namespace nano_fmm
