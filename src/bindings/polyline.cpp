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
        .def(py::init<const Eigen::Ref<const RowVectors> &, bool>(), "coords"_a,
             py::kw_only(), "is_wgs84"_a = false)
        //
        .def("polyline", &Polyline::polyline, rvp::reference_internal)
        .def("k", &Polyline::k)
        .def("is_wgs84", &Polyline::is_wgs84)
        //
        .def("range", &Polyline::range, "seg_idx"_a, py::kw_only(), "t"_a)
        .def("segment_index_t", &Polyline::segment_index_t, "range"_a)
        .def("length", &Polyline::length)
        .def("along", &Polyline::along, "range"_a, py::kw_only(),
             "extend"_a = false)
        .def("snap", &Polyline::snap, "point"_a)
        .def("slice", &Polyline::slice, py::kw_only(), "min"_a = std::nullopt,
             "max"_a = std::nullopt)
        //
        .def("segment", &Polyline::segment, "index"_a, rvp::reference_internal)
        // .def("segments", &Polyline::segments)
        .def("ranges", &Polyline::ranges, rvp::reference_internal)
        //
        ;
}
} // namespace nano_fmm
