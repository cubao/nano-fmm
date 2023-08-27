#include <pybind11/eigen.h>
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include "nano_fmm/heap.hpp"

namespace nano_fmm
{
namespace py = pybind11;
using namespace pybind11::literals;
using rvp = py::return_value_policy;

void bind_heap(py::module &m)
{
    py::class_<HeapNode>(m, "HeapNode", py::module_local()) //
        .def(py::init<int64_t, double>(), "index"_a, "value"_a)
        .def_property(
            "index", [](const HeapNode &self) { return self.index; },
            [](HeapNode &self, int64_t index) { self.index = index; })
        .def_property(
            "value", [](const HeapNode &self) { return self.value; },
            [](HeapNode &self, double value) { self.value = value; })
        //
        ;
    py::class_<Heap>(m, "Heap", py::module_local()) //
        .def(py::init<>())
        //
        .def("push", &Heap::push, "index"_a, "value"_a)
        .def("pop", &Heap::pop)
        .def("top", &Heap::top)
        .def("empty", &Heap::empty)
        .def("size", &Heap::size)
        .def("contain_node", &Heap::contain_node, "index"_a);
}
} // namespace nano_fmm
