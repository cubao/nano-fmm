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
        .def("add_road", &Network::add_road, "geom"_a, py::kw_only(), "id"_a)
        .def("add_link", &Network::add_link, "source_road"_a, "target_road"_a)
        .def("remove_road", &Network::remove_road, "id"_a)
        .def("remove_link", &Network::remove_link, //
             "source_road"_a, "target_road"_a)
        .def("prev_roads", &Network::prev_roads, "id"_a)
        .def("next_roads", &Network::next_roads, "id"_a)
        .def("roads", &Network::roads)
        //
        .def("query", &Network::query, "position"_a, //
             py::kw_only(), "radius"_a, "k"_a = std::nullopt)
        //
        .def_static("load", &Network::load, "path"_a)
        .def("dump", &Network::dump, "path"_a, py::kw_only(),
             "with_config"_a = true)
        //
        .def("build_ubodt", &Network::build_ubodt, "thresh"_a = std::nullopt)
        .def("load_ubodt", &Network::load_ubodt, "path"_a)
        .def("dump_ubodt", &Network::dump_ubodt, "path"_a, py::kw_only(),
             "thresh"_a = std::nullopt)
        //
        ;
}
} // namespace nano_fmm
