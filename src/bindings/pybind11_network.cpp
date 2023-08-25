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
    py::class_<ProjectedPoint>(m, "ProjectedPoint", py::module_local()) //
        .def(py::init<const Eigen::Vector3d &, double, int64_t, double>(),
             "position"_a = Eigen::Vector3d(0, 0, 0), "distance"_a = 0.0,
             "road_id"_a = 0, "offset"_a = 0.0)
        //
        .def_property_readonly(
            "position",
            [](const ProjectedPoint &self) { return self.position_; })
        .def_property_readonly(
            "distance",
            [](const ProjectedPoint &self) { return self.distance_; })
        .def_property_readonly(
            "road_id", [](const ProjectedPoint &self) { return self.road_id_; })
        .def_property_readonly(
            "offset", [](const ProjectedPoint &self) { return self.offset_; })
        //
        ;

    py::class_<UBODT>(m, "UBODT", py::module_local()) //
        .def(py::init<>())
        //
        .def_property_readonly("origin",
                               [](const UBODT &self) { return self.origin_; })
        .def_property_readonly(
            "destination", [](const UBODT &self) { return self.destination_; });
    //
    ;

    py::class_<Network>(m, "Network", py::module_local()) //
                                                          //
        .def(py::init<bool>(), py::kw_only(), "is_wgs84"_a = false)
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
        .def("road", &Network::road, "road_id"_a, rvp::reference_internal)
        .def("query",
             py::overload_cast<const Eigen::Vector3d &, //
                               double,                  //
                               std::optional<int>,      //
                               std::optional<double>>(&Network::query,
                                                      py::const_),
             "position"_a,         //
             py::kw_only(),        //
             "radius"_a,           //
             "k"_a = std::nullopt, //
             "z_max_offset"_a = std::nullopt)
        .def("query",
             py::overload_cast<const Eigen::Vector4d &>(&Network::query,
                                                        py::const_),
             "bbox"_a)
        //
        .def("build", &Network::build)
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
