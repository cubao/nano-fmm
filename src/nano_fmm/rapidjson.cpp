#include "nano_fmm/rapidjson_helpers.hpp"
#include "nano_fmm/network/projected_point.hpp"
#include "nano_fmm/network/ubodt.hpp"
#include "nano_fmm/network.hpp"

namespace nano_fmm
{
template <typename T> struct HAS_FROM_RAPIDJSON
{
    template <typename U, U &(U::*)(const RapidjsonValue &)> struct SFINAE
    {
    };
    template <typename U> static char Test(SFINAE<U, &U::from_rapidjson> *);
    template <typename U> static int Test(...);
    static const bool Has = sizeof(Test<T>(0)) == sizeof(char);
};

template <typename T> struct HAS_TO_RAPIDJSON
{
    template <typename U, RapidjsonValue (U::*)(RapidjsonAllocator &) const>
    struct SFINAE
    {
    };
    template <typename U> static char Test(SFINAE<U, &U::to_rapidjson> *);
    template <typename U> static int Test(...);
    static const bool Has = sizeof(Test<T>(0)) == sizeof(char);
};

template <typename T, std::enable_if_t<!HAS_FROM_RAPIDJSON<T>::Has, int> = 0>
T from_rapidjson(const RapidjsonValue &json);
template <typename T> RapidjsonValue to_rapidjson(T &&t)
{
    RapidjsonAllocator allocator;
    return to_rapidjson(std::forward<T>(t), allocator);
}

template <typename T, std::enable_if_t<HAS_FROM_RAPIDJSON<T>::Has, int> = 0>
T from_rapidjson(const RapidjsonValue &json)
{
    T t;
    t.from_rapidjson(json);
    return t;
}

template <typename T, std::enable_if_t<HAS_TO_RAPIDJSON<T>::Has, int> = 0>
RapidjsonValue to_rapidjson(const T &t, RapidjsonAllocator &allocator)
{
    return t.to_rapidjson(allocator);
}

// serialization for each types
template <> int64_t from_rapidjson(const RapidjsonValue &json)
{
    return json.GetInt64();
}
inline RapidjsonValue to_rapidjson(int64_t value, RapidjsonAllocator &allocator)
{
    return RapidjsonValue(value);
}

template <> double from_rapidjson(const RapidjsonValue &json)
{
    return json.GetDouble();
}
inline RapidjsonValue to_rapidjson(double value, RapidjsonAllocator &allocator)
{
    return RapidjsonValue(value);
}

template <> Eigen::Vector3d from_rapidjson(const RapidjsonValue &json)
{
    return {json[0].GetDouble(), json[1].GetDouble(),
            json.Size() > 2 ? json[2].GetDouble() : 0.0};
}
inline RapidjsonValue to_rapidjson(const Eigen::Vector3d &value,
                                   RapidjsonAllocator &allocator)
{
    RapidjsonValue arr(rapidjson::kArrayType);
    arr.Reserve(3, allocator);
    arr.PushBack(RapidjsonValue(value[0]), allocator);
    arr.PushBack(RapidjsonValue(value[1]), allocator);
    arr.PushBack(RapidjsonValue(value[2]), allocator);
    return arr;
}

template <> RowVectors from_rapidjson(const RapidjsonValue &json)
{
    const int N = json.Size();
    RowVectors xyzs(N, 3);
    for (int i = 0; i < N; ++i) {
        xyzs(i, 0) = json[i][0].GetDouble();
        xyzs(i, 1) = json[i][1].GetDouble();
        xyzs(i, 2) = json[i].Size() > 2 ? json[i][2].GetDouble() : 0.0;
    }
    return xyzs;
}
inline RapidjsonValue to_rapidjson(const RowVectors &value,
                                   RapidjsonAllocator &allocator)
{
    RapidjsonValue xyzs(rapidjson::kArrayType);
    const int N = value.rows();
    xyzs.Reserve(N, allocator);
    for (int i = 0; i < N; ++i) {
        RapidjsonValue xyz(rapidjson::kArrayType);
        xyz.Reserve(3, allocator);
        xyz.PushBack(RapidjsonValue(value(i, 0)), allocator);
        xyz.PushBack(RapidjsonValue(value(i, 1)), allocator);
        xyz.PushBack(RapidjsonValue(value(i, 2)), allocator);
        xyzs.PushBack(xyz, allocator);
    }
    return xyzs;
}

template <> std::vector<int64_t> from_rapidjson(const RapidjsonValue &json)
{
    const int N = json.Size();
    std::vector<int64_t> index;
    index.reserve(N);
    for (int i = 0; i < N; ++i) {
        index.push_back(json[i].GetInt64());
    }
    return index;
}
inline RapidjsonValue to_rapidjson(const std::vector<int64_t> &value,
                                   RapidjsonAllocator &allocator)
{
    RapidjsonValue xyzs(rapidjson::kArrayType);
    const int N = value.size();
    xyzs.Reserve(N, allocator);
    for (int i = 0; i < N; ++i) {
        xyzs.PushBack(RapidjsonValue(value[i]), allocator);
    }
    return xyzs;
}

// helper macros
#define TO_RAPIDJSON(var, json, allocator, key)                                \
    json.AddMember(#key, nano_fmm::to_rapidjson(var.key(), allocator),         \
                   allocator);
#define FROM_RAPIDJSON(var, json, json_end, key)                               \
    auto key##_itr = json.FindMember(#key);                                    \
    if (json_end != key##_itr) {                                               \
        if (key##_itr->value.IsNull()) {                                       \
            var.key(std::decay<decltype(var.key())>::type());                  \
        } else {                                                               \
            var.key(nano_fmm::from_rapidjson<                                  \
                    std::decay<decltype(var.key())>::type>(key##_itr->value)); \
        }                                                                      \
    }

//  ProjectedPoint
ProjectedPoint &ProjectedPoint::from_rapidjson(const RapidjsonValue &json)
{
    auto json_end = json.MemberEnd();
    FROM_RAPIDJSON((*this), json, json_end, position)
    FROM_RAPIDJSON((*this), json, json_end, direction)
    FROM_RAPIDJSON((*this), json, json_end, distance)
    FROM_RAPIDJSON((*this), json, json_end, road_id)
    FROM_RAPIDJSON((*this), json, json_end, offset)
    return *this;
}
RapidjsonValue ProjectedPoint::to_rapidjson(RapidjsonAllocator &allocator) const
{
    RapidjsonValue json(rapidjson::kObjectType);
    TO_RAPIDJSON((*this), json, allocator, position)
    TO_RAPIDJSON((*this), json, allocator, direction)
    TO_RAPIDJSON((*this), json, allocator, distance)
    TO_RAPIDJSON((*this), json, allocator, road_id)
    TO_RAPIDJSON((*this), json, allocator, offset)
    return json;
}

// UbodtRecord
UbodtRecord &UbodtRecord::from_rapidjson(const RapidjsonValue &json)
{
    auto json_end = json.MemberEnd();
    FROM_RAPIDJSON((*this), json, json_end, source_road)
    FROM_RAPIDJSON((*this), json, json_end, target_road)
    FROM_RAPIDJSON((*this), json, json_end, source_next)
    FROM_RAPIDJSON((*this), json, json_end, target_prev)
    FROM_RAPIDJSON((*this), json, json_end, cost)
    return *this;
}
RapidjsonValue UbodtRecord::to_rapidjson(RapidjsonAllocator &allocator) const
{
    RapidjsonValue json(rapidjson::kObjectType);
    TO_RAPIDJSON((*this), json, allocator, source_road)
    TO_RAPIDJSON((*this), json, allocator, target_road)
    TO_RAPIDJSON((*this), json, allocator, source_next)
    TO_RAPIDJSON((*this), json, allocator, target_prev)
    TO_RAPIDJSON((*this), json, allocator, cost)
    return json;
}

Config &Config::from_rapidjson(const RapidjsonValue &json)
{
    auto json_end = json.MemberEnd();
    return *this;
}
RapidjsonValue Config::to_rapidjson(RapidjsonAllocator &allocator) const
{
    RapidjsonValue json(rapidjson::kObjectType);
    return json;
}

Network &Network::from_geojson(const RapidjsonValue &json)
{
    auto json_end = json.MemberEnd();
    auto config_itr = json.FindMember("config");
    if (config_itr != json_end) {
        config_ = nano_fmm::from_rapidjson<Config>(config_itr->value);
    }
    return *this;
}
RapidjsonValue Network::to_geojson(RapidjsonAllocator &allocator) const
{
    RapidjsonValue features(rapidjson::kArrayType);
    features.Reserve(roads_.size(), allocator);

    std::vector<int64_t> road_ids;
    for (auto &pair : roads_) {
        road_ids.push_back(pair.first);
    }
    std::sort(road_ids.begin(), road_ids.end());
    for (auto id : road_ids) {
        auto &ruler = roads_.at(id);
        RapidjsonValue geometry(rapidjson::kObjectType);
        geometry.AddMember("type", "LineString", allocator);
        geometry.AddMember("coordinates",
                           nano_fmm::to_rapidjson(ruler.polyline(), allocator),
                           allocator);
        RapidjsonValue feature(rapidjson::kObjectType);
        feature.AddMember("type", "Feature", allocator);
        feature.AddMember("geometry", geometry, allocator);
        RapidjsonValue properties(rapidjson::kObjectType);
        properties.AddMember("type", "road", allocator);
        properties.AddMember("id", RapidjsonValue(id), allocator);
        auto nexts_itr = nexts_.find(id);
        if (nexts_itr == nexts_.end()) {
            properties.AddMember(
                "nexts",
                nano_fmm::to_rapidjson(std::vector<int64_t>{}, allocator),
                allocator);
        } else {
            auto ids = std::vector<int64_t>(nexts_itr->second.begin(),
                                            nexts_itr->second.end());
            std::sort(ids.begin(), ids.end());
            properties.AddMember(
                "nexts", nano_fmm::to_rapidjson(ids, allocator), allocator);
        }
        auto prevs_itr = prevs_.find(id);
        if (prevs_itr == prevs_.end()) {
            properties.AddMember(
                "prevs",
                nano_fmm::to_rapidjson(std::vector<int64_t>{}, allocator),
                allocator);
        } else {
            auto ids = std::vector<int64_t>(prevs_itr->second.begin(),
                                            prevs_itr->second.end());
            std::sort(ids.begin(), ids.end());
            properties.AddMember(
                "prevs", nano_fmm::to_rapidjson(ids, allocator), allocator);
        }
        feature.AddMember("properties", properties, allocator);
        features.PushBack(feature, allocator);
    }

    RapidjsonValue geojson(rapidjson::kObjectType);
    geojson.AddMember("type", "FeatureCollection", allocator);
    geojson.AddMember("features", features, allocator);
    geojson.AddMember("config", config_.to_rapidjson(allocator), allocator);
    return geojson;
}

Network &Network::from_rapidjson(const RapidjsonValue &json)
{
    auto json_end = json.MemberEnd();
    return *this;
}
RapidjsonValue Network::to_rapidjson(RapidjsonAllocator &allocator) const
{
    RapidjsonValue json(rapidjson::kObjectType);
    return json;
}

} // namespace nano_fmm
