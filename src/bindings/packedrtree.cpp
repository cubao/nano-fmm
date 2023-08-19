#include <pybind11/eigen.h>
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include "packedrtree.h"
#include "spdlog/spdlog.h"

namespace nano_fmm
{
namespace py = pybind11;
using namespace pybind11::literals;
using rvp = py::return_value_policy;

void bind_packedrtree(py::module &m)
{
    using namespace FlatGeobuf;
    py::class_<NodeItem>(m, "NodeItem", py::module_local()) //
        .def(py::init<>())
        .def(py::init<double, double, double, double, uint64_t>(), "minX"_a,
             "minY"_a, "maxX"_a, "maxY"_a, "offset"_a)
        .def_readwrite("minX", &NodeItem::minX)
        .def_readwrite("minY", &NodeItem::minY)
        .def_readwrite("maxX", &NodeItem::maxX)
        .def_readwrite("maxY", &NodeItem::maxY)
        .def_readwrite("offset", &NodeItem::offset)
        .def("width", &NodeItem::width)
        .def("height", &NodeItem::height)
        .def_static("sum", &NodeItem::sum, "a"_a, "b"_a)
        .def_static("create", &NodeItem::create, "offset"_a = 0)
        .def("expand", &NodeItem::expand, "r"_a)
        .def("intersects", &NodeItem::intersects, "r"_a)
        .def("toVector", &NodeItem::toVector)
        .def("__repr__",
             [](const NodeItem &n) {
                 return fmt::format(
                     "NodeItem(min=[{},{}],max=[{},{}],offset={})", n.minX,
                     n.minY, n.maxX, n.maxY, n.offset);
             })
        //
        ;
    py::class_<Item>(m, "Item", py::module_local()) //
        .def(py::init<>())
        .def_readwrite("nodeItem", &Item::nodeItem)
        //
        ;
    py::class_<SearchResultItem>(m, "SearchResultItem", py::module_local()) //
        .def(py::init<>())
        .def_readwrite("offset", &SearchResultItem::offset)
        .def_readwrite("index", &SearchResultItem::index)
        //
        ;

    m.def("hilbert", py::overload_cast<uint32_t, uint32_t>(&hilbert), //
    "x"_a, "y"_a)
        //
        ;

    py::class_<PackedRTree>(m, "PackedRTree", py::module_local()) //
        .def(py::init<const std::vector<NodeItem> &, const NodeItem &,
                      const uint16_t>(),
             "nodes"_a, "extent"_a, "nodeSize"_a = 16)
        .def("search", &PackedRTree::search, "minX"_a, "minY"_a, "maxX"_a,
             "maxY"_a)
        .def_static("generateLevelBounds", &PackedRTree::generateLevelBounds,
                    "numItems"_a, "nodeSize"_a)
        .def("size", py::overload_cast<>(&PackedRTree::size, py::const_))
        .def("getExtent", &PackedRTree::getExtent)

        //
        ;
}
} // namespace nano_fmm
