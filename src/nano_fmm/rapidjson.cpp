#include "nano_fmm/rapidjson_helpers.hpp"
#include "nano_fmm/network/projected_point.hpp"
#include "nano_fmm/network/ubodt.hpp"

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

template <typename T> T from_rapidjson(const RapidjsonValue &json);
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
    return {json[0].GetDouble(), json[1].GetDouble(), json[2].GetDouble()};
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
} // namespace nano_fmm
