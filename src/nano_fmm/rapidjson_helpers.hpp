#pragma once

#include <fstream>
#include <iostream>
#include <set>

#include "rapidjson/error/en.h"
#include "rapidjson/filereadstream.h"
#include "rapidjson/filewritestream.h"
#include "rapidjson/prettywriter.h"
#include "rapidjson/stringbuffer.h"

#include "nano_fmm/types.hpp"

namespace nano_fmm
{
constexpr const auto RJFLAGS = rapidjson::kParseDefaultFlags |      //
                               rapidjson::kParseCommentsFlag |      //
                               rapidjson::kParseFullPrecisionFlag | //
                               rapidjson::kParseTrailingCommasFlag;

inline RapidjsonValue deepcopy(const RapidjsonValue &json,
                               RapidjsonAllocator &allocator)
{
    RapidjsonValue copy;
    copy.CopyFrom(json, allocator);
    return copy;
}
inline RapidjsonValue deepcopy(const RapidjsonValue &json)
{
    RapidjsonAllocator allocator;
    return deepcopy(json, allocator);
}

template <typename T> RapidjsonValue int_to_rapidjson(T const &num)
{
    if (sizeof(T) < sizeof(int64_t)) {
        return std::is_signed<T>::value
                   ? RapidjsonValue(static_cast<int32_t>(num))
                   : RapidjsonValue(static_cast<uint32_t>(num));
    } else {
        return std::is_signed<T>::value
                   ? RapidjsonValue(static_cast<int64_t>(num))
                   : RapidjsonValue(static_cast<uint64_t>(num));
    }
}

inline void sort_keys_inplace(RapidjsonValue &json)
{
    if (json.IsArray()) {
        for (auto &e : json.GetArray()) {
            sort_keys_inplace(e);
        }
    } else if (json.IsObject()) {
        auto obj = json.GetObject();
        // https://rapidjson.docsforge.com/master/sortkeys.cpp/
        std::sort(obj.MemberBegin(), obj.MemberEnd(), [](auto &lhs, auto &rhs) {
            return strcmp(lhs.name.GetString(), rhs.name.GetString()) < 0;
        });
        for (auto &kv : obj) {
            sort_keys_inplace(kv.value);
        }
    }
}

inline void round_rapidjson(RapidjsonValue &json, double scale, int depth = 1,
                            const std::vector<std::string> &skip_keys = {})
{
    if (--depth < 0) {
        return;
    }
    if (json.IsArray()) {
        for (auto &e : json.GetArray()) {
            round_rapidjson(e, scale, depth, skip_keys);
        }
    } else if (json.IsObject()) {
        auto obj = json.GetObject();
        for (auto &kv : obj) {
            if (!skip_keys.empty() &&
                std::find(skip_keys.begin(), skip_keys.end(),
                          std::string(kv.name.GetString(),
                                      kv.name.GetStringLength())) !=
                    skip_keys.end()) {
                continue;
            }
            round_rapidjson(kv.value, scale, depth, skip_keys);
        }
    } else if (json.IsDouble()) {
        // see round_coords in geojson_helpers
        json.SetDouble(std::floor(json.GetDouble() * scale + 0.5) / scale);
    }
}

inline void round_non_geojson(RapidjsonValue &json, double scale)
{
    if (json.IsObject()) {
        auto itr = json.FindMember("type");
        if (itr != json.MemberEnd() && itr->value.IsString()) {
            const auto type = std::string(itr->value.GetString(),
                                          itr->value.GetStringLength());
            if (                             //
                type == "FeatureCollection"  //
                || type == "Feature"         //
                || type == "Point"           //
                || type == "MultiPoint"      //
                || type == "LineString"      //
                || type == "MultiLineString" //
                || type == "Polygon"         //
                || type == "MultiPolygon"    //
                || type == "GeometryCollection") {
                return;
            }
        }
    }
    round_rapidjson(json, scale, INT_MAX);
}

inline void round_geojson_non_geometry(RapidjsonValue &json, double scale)
{
    if (!json.IsObject()) {
        return;
    }
    auto itr = json.FindMember("type");
    if (itr == json.MemberEnd() || !itr->value.IsString()) {
        return;
    }
    const auto type =
        std::string(itr->value.GetString(), itr->value.GetStringLength());
    if (type == "Feature") {
        round_rapidjson(json, scale, INT_MAX, {"geometry"});
        round_geojson_non_geometry(json["geometry"], scale);
    } else if (type == "FeatureCollection") {
        round_rapidjson(json, scale, INT_MAX, {"features"});
        for (auto &f : json["features"].GetArray()) {
            round_geojson_non_geometry(f, scale);
        }
    } else if (type == "Point" || type == "MultiPoint" ||
               type == "LineString" || type == "MultiLineString" ||
               type == "Polygon" || type == "MultiPolygon") {
        round_rapidjson(json, scale, INT_MAX, {"coordinates"});
    } else if (type == "GeometryCollection") {
        round_rapidjson(json, scale, INT_MAX, {"geometries"});
        for (auto &g : json["geometries"].GetArray()) {
            round_geojson_non_geometry(g, scale);
        }
    }
}

inline void __round_geojson_geometry(RapidjsonValue &json,
                                     const Eigen::Vector3d &scale)
{
    if (!json.IsArray() || json.Empty()) {
        return;
    }
    if (!json[0].IsNumber()) {
        for (auto &e : json.GetArray()) {
            __round_geojson_geometry(e, scale);
        }
        return;
    }
    const int N = std::min(3, (int)json.Size());
    for (int i = 0; i < N; ++i) {
        if (json[i].IsDouble()) {
            json[i].SetDouble(std::floor(json[i].GetDouble() * scale[i] + 0.5) /
                              scale[i]);
        }
    }
}

inline void round_geojson_geometry(RapidjsonValue &json,
                                   const Eigen::Vector3d &scale)
{
    if (!json.IsObject()) {
        return;
    }
    auto itr = json.FindMember("type");
    if (itr == json.MemberEnd() || !itr->value.IsString()) {
        return;
    }
    const auto type =
        std::string(itr->value.GetString(), itr->value.GetStringLength());
    if (type == "Feature") {
        round_geojson_geometry(json["geometry"], scale);
    } else if (type == "FeatureCollection") {
        for (auto &f : json["features"].GetArray()) {
            round_geojson_geometry(f["geometry"], scale);
        }
    } else if (type == "Point" || type == "MultiPoint" ||
               type == "LineString" || type == "MultiLineString" ||
               type == "Polygon" || type == "MultiPolygon") {
        __round_geojson_geometry(json["coordinates"], scale);
    } else if (type == "GeometryCollection") {
        for (auto &g : json["geometries"].GetArray()) {
            round_geojson_geometry(g, scale);
        }
    }
}

inline void denoise_double_0_rapidjson(RapidjsonValue &json)
{
    if (json.IsArray()) {
        for (auto &e : json.GetArray()) {
            denoise_double_0_rapidjson(e);
        }
    } else if (json.IsObject()) {
        auto obj = json.GetObject();
        for (auto &kv : obj) {
            denoise_double_0_rapidjson(kv.value);
        }
    } else if (json.IsDouble()) {
        double d = json.GetDouble();
        if (std::floor(d) == d) {
            if (d >= 0) {
                auto i = static_cast<uint64_t>(d);
                if (i == d) {
                    json.SetUint64(i);
                }
            } else {
                auto i = static_cast<int64_t>(d);
                if (i == d) {
                    json.SetInt64(i);
                }
            }
        }
    }
}

inline bool __all_is_z0(RapidjsonValue &json)
{
    // [x, y, 0.0], [[x, y, 0.0], ...], [[[x, y, 0.0], ..], ..]
    if (!json.IsArray()) {
        return false;
    }
    if (json.Empty()) {
        return true;
    }
    if (!json[0].IsNumber()) {
        for (auto &e : json.GetArray()) {
            if (!__all_is_z0(e)) {
                return false;
            }
        }
        return true;
    }
    if (json.Size() != 3 || !json[2].IsNumber()) {
        return false;
    }
    return json[2].GetDouble() == 0.0;
}

inline void __strip_geometry_z_0(RapidjsonValue &json)
{
    if (!json.IsArray() || json.Empty()) {
        return;
    }
    if (!json[0].IsNumber()) {
        for (auto &e : json.GetArray()) {
            __strip_geometry_z_0(e);
        }
        return;
    }
    if (json.Size() == 3) {
        json.PopBack();
    }
}

inline void strip_geometry_z_0(RapidjsonValue &json)
{
    if (json.IsObject()) {
        auto itr = json.FindMember("type");
        if (itr == json.MemberEnd() || !itr->value.IsString()) {
            return;
        }
        const auto type =
            std::string(itr->value.GetString(), itr->value.GetStringLength());
        if (type == "Feature") {
            strip_geometry_z_0(json["geometry"]);
        } else if (type == "FeatureCollection") {
            for (auto &f : json["features"].GetArray()) {
                strip_geometry_z_0(f["geometry"]);
            }
        } else if (type == "Point" || type == "MultiPoint" ||
                   type == "LineString" || type == "MultiLineString" ||
                   type == "Polygon" || type == "MultiPolygon") {
            strip_geometry_z_0(json["coordinates"]);
        } else if (type == "GeometryCollection") {
            for (auto &g : json["geometries"].GetArray()) {
                strip_geometry_z_0(g);
            }
        }
        return;
    }

    if (!__all_is_z0(json)) {
        return;
    }
    __strip_geometry_z_0(json);
}

inline void
normalize_json(RapidjsonValue &json,                              //
               bool sort_keys = true,                             //
               std::optional<int> round_geojson_non_geometry = 3, //
               const std::optional<std::array<int, 3>> &round_geojson_geometry =
                   std::array<int, 3>{8, 8, 3},          //
               std::optional<int> round_non_geojson = 3, //
               bool denoise_double_0 = true,             //
               bool strip_geometry_z_0 = true)
{
    if (sort_keys) {
        sort_keys_inplace(json);
    }
    if (round_geojson_non_geometry) {
        double scale = std::pow(10.0, *round_geojson_non_geometry);
        nano_fmm::round_geojson_non_geometry(json, scale);
    }
    if (round_geojson_geometry) {
        auto &precision = *round_geojson_geometry;
        nano_fmm::round_geojson_geometry(json, {std::pow(10.0, precision[0]),
                                                std::pow(10.0, precision[1]),
                                                std::pow(10.0, precision[2])});
    }
    if (round_non_geojson) {
        double scale = std::pow(10.0, *round_non_geojson);
        nano_fmm::round_non_geojson(json, scale);
    }
    if (strip_geometry_z_0) {
        nano_fmm::strip_geometry_z_0(json);
    }
    if (denoise_double_0) {
        denoise_double_0_rapidjson(json);
    }
}

inline RapidjsonValue sort_keys(const RapidjsonValue &json)
{
    RapidjsonAllocator allocator;
    RapidjsonValue copy;
    copy.CopyFrom(json, allocator);
    sort_keys_inplace(copy);
    return copy;
}

inline RapidjsonValue load_json(const std::string &path)
{
    FILE *fp = fopen(path.c_str(), "rb");
    if (!fp) {
        throw std::runtime_error("can't open for reading: " + path);
    }
    char readBuffer[65536];
    rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));
    RapidjsonDocument d;
    d.ParseStream<RJFLAGS>(is);
    fclose(fp);
    return RapidjsonValue{std::move(d.Move())};
}
inline bool dump_json(const std::string &path, const RapidjsonValue &json,
                      bool indent = false, bool sort_keys = false)
{
    FILE *fp = fopen(path.c_str(), "wb");
    if (!fp) {
        std::cerr << "can't open for writing: " + path << std::endl;
        return false;
    }
    using namespace rapidjson;
    char writeBuffer[65536];
    bool succ = false;
    FileWriteStream os(fp, writeBuffer, sizeof(writeBuffer));
    if (indent) {
        PrettyWriter<FileWriteStream> writer(os);
        if (sort_keys) {
            succ = nano_fmm::sort_keys(json).Accept(writer);
        } else {
            succ = json.Accept(writer);
        }
    } else {
        Writer<FileWriteStream> writer(os);
        if (sort_keys) {
            succ = nano_fmm::sort_keys(json).Accept(writer);
        } else {
            succ = json.Accept(writer);
        }
    }
    fclose(fp);
    return succ;
}

inline RapidjsonValue loads(const std::string &json)
{
    RapidjsonDocument d;
    rapidjson::StringStream ss(json.data());
    d.ParseStream<RJFLAGS>(ss);
    if (d.HasParseError()) {
        throw std::invalid_argument(
            "invalid json, offset: " + std::to_string(d.GetErrorOffset()) +
            ", error: " + rapidjson::GetParseError_En(d.GetParseError()));
    }
    return RapidjsonValue{std::move(d.Move())};
}
inline std::string dumps(const RapidjsonValue &json, bool indent = false,
                         bool sort_keys = false)
{
    if (sort_keys) {
        return dumps(nano_fmm::sort_keys(json), indent, !sort_keys);
    }
    rapidjson::StringBuffer buffer;
    if (indent) {
        rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buffer);
        json.Accept(writer);
    } else {
        rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
        json.Accept(writer);
    }
    return buffer.GetString();
}

inline bool __bool__(const RapidjsonValue &self)
{
    if (self.IsArray()) {
        return !self.Empty();
    } else if (self.IsObject()) {
        return !self.ObjectEmpty();
    } else if (self.IsString()) {
        return self.GetStringLength() != 0u;
    } else if (self.IsBool()) {
        return self.GetBool();
    } else if (self.IsNumber()) {
        if (self.IsUint64()) {
            return self.GetUint64() != 0;
        } else if (self.IsInt64()) {
            return self.GetInt64() != 0;
        } else {
            return self.GetDouble() != 0.0;
        }
    }
    return !self.IsNull();
}

inline int __len__(const RapidjsonValue &self)
{
    if (self.IsArray()) {
        return self.Size();
    } else if (self.IsObject()) {
        return self.MemberCount();
    }
    return 0;
}
} // namespace nano_fmm
