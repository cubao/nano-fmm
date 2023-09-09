#include <pybind11/eigen.h>
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include "nano_fmm/network.hpp"
#include "nano_fmm/indexer.hpp"
#include "spdlog/spdlog.h"

namespace nano_fmm
{
namespace py = pybind11;
using namespace pybind11::literals;
using rvp = py::return_value_policy;

void bind_network(py::module &m)
{
    py::class_<ProjectedPoint>(m, "ProjectedPoint", py::module_local()) //
        .def(py::init<const Eigen::Vector3d &,                          //
                      const Eigen::Vector3d &,                          //
                      double,                                           //
                      int64_t,                                          //
                      double>(),
             py::kw_only(),                            //
             "position"_a = Eigen::Vector3d(0, 0, 0),  //
             "direction"_a = Eigen::Vector3d(0, 0, 1), //
             "distance"_a = 0.0,                       //
             "road_id"_a = 0,                          //
             "offset"_a = 0.0)
        //
        .def_property_readonly(
            "position",
            [](const ProjectedPoint &self) -> const Eigen::Vector3d {
                return self.position();
            })
        .def_property_readonly(
            "direction",
            [](const ProjectedPoint &self) -> const Eigen::Vector3d {
                return self.direction();
            })
        .def_property_readonly(
            "distance",
            [](const ProjectedPoint &self) { return self.distance(); })
        .def_property_readonly(
            "road_id",
            [](const ProjectedPoint &self) { return self.road_id(); })
        .def_property_readonly(
            "offset", [](const ProjectedPoint &self) { return self.offset(); })
        //
        .def("from_rapidjson", &ProjectedPoint::from_rapidjson, "json"_a)
        .def("to_rapidjson",
             py::overload_cast<>(&ProjectedPoint::to_rapidjson, py::const_))
        //
        .def("__repr__", [](const ProjectedPoint &self) {
            auto &p = self.position();
            auto &d = self.direction();
            return fmt::format("ProjectedPoint(pos=[{},{},{}],dir=[{},{},{}],"
                               "dist={},road={},offset={})",
                               p[0], p[1], p[2], //
                               d[0], d[1], d[2], //
                               self.distance(), self.road_id(), self.offset());
        });
    //
    ;

    py::class_<UbodtRecord>(m, "UbodtRecord", py::module_local()) //
        .def(py::init<int64_t, int64_t, int64_t, int64_t, double>(),
             py::kw_only(),
             "source_road"_a = 0, //
             "target_road"_a = 0, //
             "source_next"_a = 0, //
             "target_prev"_a = 0, //
             "cost"_a = 0.0)
        //
        .def_property_readonly(
            "source_road",
            [](const UbodtRecord &self) { return self.source_road(); })
        .def_property_readonly(
            "target_road",
            [](const UbodtRecord &self) { return self.target_road(); })
        .def_property_readonly(
            "source_next",
            [](const UbodtRecord &self) { return self.source_next(); })
        .def_property_readonly(
            "target_prev",
            [](const UbodtRecord &self) { return self.target_prev(); })
        .def_property_readonly(
            "cost", [](const UbodtRecord &self) { return self.cost(); })
        //
        .def(py::self == py::self)
        .def(py::self < py::self)
        //
        .def("from_rapidjson", &UbodtRecord::from_rapidjson, "json"_a)
        .def("to_rapidjson",
             py::overload_cast<>(&UbodtRecord::to_rapidjson, py::const_))
        //
        .def("__repr__", [](const UbodtRecord &self) {
            return fmt::format(
                "UbodtRecord(s->t=[{}->{}], cost:{}, sn:{},tp:{})",
                self.source_road(), self.target_road(), //
                self.cost(),                            //
                self.source_next(), self.target_prev());
        });
    //
    ;

    py::class_<Network>(m, "Network", py::module_local()) //
                                                          //
        .def(py::init<bool>(), py::kw_only(), "is_wgs84"_a = true)
        //
        .def("add_road", &Network::add_road, "geom"_a, py::kw_only(), "id"_a)
        .def("add_link", &Network::add_link, "source_road"_a, "target_road"_a,
             py::kw_only(), "check_road"_a = false)
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
             "position"_a,                    //
             py::kw_only(),                   //
             "radius"_a,                      //
             "k"_a = std::nullopt,            //
             "z_max_offset"_a = std::nullopt, //
             py::call_guard<py::gil_scoped_release>())
        .def("query",
             py::overload_cast<const Eigen::Vector4d &>(&Network::query,
                                                        py::const_),
             "bbox"_a, //
             py::call_guard<py::gil_scoped_release>())
        //
        .def("build", &Network::build, "execution_policy"_a = 2,
             py::call_guard<py::gil_scoped_release>())
        //
        .def_static("load", &Network::load, "path"_a)
        .def("dump", &Network::dump, "path"_a, py::kw_only(), "indent"_a = true,
             "as_geojson"_a = true)
        //
        .def("build_ubodt",
             py::overload_cast<std::optional<double>>(&Network::build_ubodt,
                                                      py::const_),
             py::kw_only(), "thresh"_a = std::nullopt)
        .def("build_ubodt",
             py::overload_cast<const std::vector<int64_t> &,
                               std::optional<double>>(&Network::build_ubodt,
                                                      py::const_),
             "roads"_a, py::kw_only(), "thresh"_a = std::nullopt)
        .def("clear_ubodt", &Network::clear_ubodt)
        .def("load_ubodt",
             py::overload_cast<const std::vector<UbodtRecord> &>(
                 &Network::load_ubodt),
             "rows"_a)
        .def("load_ubodt",
             py::overload_cast<const std::string &>(&Network::load_ubodt),
             "path"_a)
        .def("dump_ubodt", &Network::dump_ubodt, "path"_a, py::kw_only(),
             "thresh"_a = std::nullopt)
        //
        .def("to_2d", &Network::to_2d)
        //
        ;

    py::class_<Indexer>(m, "Indexer", py::module_local()) //
        .def(py::init<>())
        .def("id", py::overload_cast<int64_t>(&Indexer::id), "id"_a)
        .def("id", py::overload_cast<const std::string &>(&Indexer::id), "id"_a)
        .def("index",
             py::overload_cast<const std::string &, int64_t>(&Indexer::index),
             "str_id"_a, "int_id"_a)
        .def("index", py::overload_cast<>(&Indexer::index, py::const_))
        //
        .def("from_rapidjson", &Indexer::from_rapidjson, "json"_a)
        .def("to_rapidjson",
             py::overload_cast<>(&Indexer::to_rapidjson, py::const_))
        //
        //
        ;
}
} // namespace nano_fmm
