#include <pybind11/pybind11.h>

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

int add(int i, int j) { return i + j; }

namespace py = pybind11;

namespace nano_fmm
{
void bind_benchmarks(py::module &m);
void bind_network(py::module &m);
void bind_packedrtree(py::module &m);
void bind_polyline(py::module &m);
void bind_randoms(py::module &m);
void bind_rapidjson(py::module &m);
void bind_utils(py::module &m);
} // namespace nano_fmm

PYBIND11_MODULE(_nano_fmm, m)
{
    m.def("add", &add, R"pbdoc(
        Add two numbers

        Some other explanation about the add function.
    )pbdoc");

#ifdef VERSION_INFO
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif

    auto utils = m.def_submodule("utils");
    auto benchmarks = m.def_submodule("benchmarks");
    auto flatbush = m.def_submodule("flatbush");

    nano_fmm::bind_benchmarks(benchmarks);
    nano_fmm::bind_network(m);
    nano_fmm::bind_packedrtree(flatbush);
    nano_fmm::bind_polyline(m);
    nano_fmm::bind_randoms(m);
    nano_fmm::bind_rapidjson(m);
    nano_fmm::bind_utils(utils);
}
