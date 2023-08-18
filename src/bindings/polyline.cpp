#include <pybind11/eigen.h>
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include "nano_fmm/polyline.hpp"

namespace nano_fmm
{
namespace py = pybind11;
using namespace pybind11::literals;
using rvp = py::return_value_policy;

void bind_polyline(py::module &m)
{
    py::class_<LineSegment>(m, "LineSegment", py::module_local())      //
        .def(py::init<const Eigen::Vector3d, const Eigen::Vector3d>(), //
             "A"_a, "B"_a)
        .def("distance", &LineSegment::distance, "P"_a)
        .def("distance2", &LineSegment::distance2, "P"_a)
        .def("nearest", &LineSegment::nearest, "P"_a)
        .def("t", &LineSegment::t, "P"_a)
        .def("interpolate", &LineSegment::interpolate, "t"_a)
        .def_property_readonly(
            "length", [](const LineSegment &self) { return self.length(); })
        .def_property_readonly(
            "length2", [](const LineSegment &self) { return self.len2; })
        .def_property_readonly("dir",
                               [](const LineSegment &self) -> Eigen::Vector3d {
                                   return self.dir();
                               })
        .def_property_readonly(
            "A",
            [](const LineSegment &self) -> const Eigen::Vector3d & {
                return self.A;
            },
            rvp::reference_internal)
        .def_property_readonly(
            "B",
            [](const LineSegment &self) -> const Eigen::Vector3d & {
                return self.B;
            },
            rvp::reference_internal)
        .def_property_readonly(
            "AB",
            [](const LineSegment &self) -> const Eigen::Vector3d & {
                return self.AB;
            },
            rvp::reference_internal)
        //
        ;

    py::class_<Polyline>(m, "Polyline", py::module_local()) //
        .def(py::init<const Eigen::Ref<const RowVectors> &,
                      const std::optional<Eigen::Vector3d>>(), //
             "coords"_a, py::kw_only(), "k"_a = std::nullopt)
        //
        .def("polyline", &Polyline::polyline, rvp::reference_internal)
        .def("scale", &Polyline::scale)
        .def("is_wgs84", &Polyline::is_wgs84)
        //
        ;
}
} // namespace nano_fmm
