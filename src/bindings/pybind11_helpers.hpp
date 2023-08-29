#pragma once

// migrated from
// https://github.com/cubao/geobuf-cpp/blob/dev/src/geobuf/pybind11_helpers.hpp

#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include "nano_fmm/types.hpp"
#include "nano_fmm/rapidjson_helpers.hpp"

namespace nano_fmm
{
namespace py = pybind11;
using namespace pybind11::literals;
using rvp = py::return_value_policy;

inline RapidjsonValue __py_int_to_rapidjson(const py::handle &obj)
{
    try {
        auto num = obj.cast<int64_t>();
        if (py::int_(num).equal(obj)) {
            return RapidjsonValue(num);
        }
    } catch (...) {
    }
    try {
        auto num = obj.cast<uint64_t>();
        if (py::int_(num).equal(obj)) {
            return RapidjsonValue(num);
        }
    } catch (...) {
    }
    throw std::runtime_error(
        "failed to convert to rapidjson, invalid integer: " +
        py::repr(obj).cast<std::string>());
}

inline RapidjsonValue to_rapidjson(const py::handle &obj,
                                   RapidjsonAllocator &allocator)
{
    if (obj.ptr() == nullptr || obj.is_none()) {
        return {};
    }
    if (py::isinstance<py::bool_>(obj)) {
        return RapidjsonValue(obj.cast<bool>());
    }
    if (py::isinstance<py::int_>(obj)) {
        return __py_int_to_rapidjson(obj);
    }
    if (py::isinstance<py::float_>(obj)) {
        return RapidjsonValue(obj.cast<double>());
    }
    if (py::isinstance<py::bytes>(obj)) {
        // https://github.com/pybind/pybind11_json/blob/master/include/pybind11_json/pybind11_json.hpp#L112
        py::module base64 = py::module::import("base64");
        auto str = base64.attr("b64encode")(obj)
                       .attr("decode")("utf-8")
                       .cast<std::string>();
        return RapidjsonValue(str.data(), str.size(), allocator);
    }
    if (py::isinstance<py::str>(obj)) {
        auto str = obj.cast<std::string>();
        return RapidjsonValue(str.data(), str.size(), allocator);
    }
    if (py::isinstance<py::tuple>(obj) || py::isinstance<py::list>(obj)) {
        RapidjsonValue arr(rapidjson::kArrayType);
        for (const py::handle &value : obj) {
            arr.PushBack(to_rapidjson(value, allocator), allocator);
        }
        return arr;
    }
    if (py::isinstance<py::dict>(obj)) {
        RapidjsonValue kv(rapidjson::kObjectType);
        for (const py::handle &key : obj) {
            auto k = py::str(key).cast<std::string>();
            kv.AddMember(RapidjsonValue(k.data(), k.size(), allocator),
                         to_rapidjson(obj[key], allocator), allocator);
        }
        return kv;
    }
    if (py::isinstance<RapidjsonValue>(obj)) {
        auto ptr = py::cast<const RapidjsonValue *>(obj);
        return deepcopy(*ptr, allocator);
    }
    throw std::runtime_error(
        "to_rapidjson not implemented for this type of object: " +
        py::repr(obj).cast<std::string>());
}

inline RapidjsonValue to_rapidjson(const py::handle &obj)
{
    RapidjsonAllocator allocator;
    return to_rapidjson(obj, allocator);
}

inline py::object to_python(const RapidjsonValue &j)
{
    if (j.IsNull()) {
        return py::none();
    } else if (j.IsBool()) {
        return py::bool_(j.GetBool());
    } else if (j.IsNumber()) {
        if (j.IsUint64()) {
            return py::int_(j.GetUint64());
        } else if (j.IsInt64()) {
            return py::int_(j.GetInt64());
        } else {
            return py::float_(j.GetDouble());
        }
    } else if (j.IsString()) {
        return py::str(std::string{j.GetString(), j.GetStringLength()});
    } else if (j.IsArray()) {
        py::list ret;
        for (const auto &e : j.GetArray()) {
            ret.append(to_python(e));
        }
        return ret;
    } else {
        py::dict ret;
        for (auto &m : j.GetObject()) {
            ret[py::str(
                std::string{m.name.GetString(), m.name.GetStringLength()})] =
                to_python(m.value);
        }
        return ret;
    }
}
} // namespace nano_fmm

#ifndef BIND_PY_FLUENT_ATTRIBUTE
#define BIND_PY_FLUENT_ATTRIBUTE(Klass, type, var)                             \
    .def(                                                                      \
        #var, [](Klass &self) -> type & { return self.var; },                  \
        rvp::reference_internal)                                               \
        .def(                                                                  \
            #var,                                                              \
            [](Klass &self, const type &v) -> Klass & {                        \
                self.var = v;                                                  \
                return self;                                                   \
            },                                                                 \
            rvp::reference_internal)
#endif
