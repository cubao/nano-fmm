#include "nano_fmm/rapidjson_helpers.hpp"
#include "nano_fmm/network/projected_point.hpp"
#include "nano_fmm/network/ubodt.hpp"

namespace nano_fmm
{

template <typename T> T from_rapidjson(const RapidjsonValue &json);
template <typename T> RapidjsonValue to_rapidjson(T &&t)
{
    RapidjsonAllocator allocator;
    return to_rapidjson(std::forward<T>(t), allocator);
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
    // FROM_RAPIDJSON((*this), json, json_end, source_road)
    return *this;
}
RapidjsonValue ProjectedPoint::to_rapidjson(RapidjsonAllocator &allocator) const
{
    RapidjsonValue json(rapidjson::kObjectType);
    return json;
}

// UbodtRecord
UbodtRecord &UbodtRecord::from_rapidjson(const RapidjsonValue &json)
{
    auto json_end = json.MemberEnd();
    FROM_RAPIDJSON((*this), json, json_end, source_road)
    return *this;
}
RapidjsonValue UbodtRecord::to_rapidjson(RapidjsonAllocator &allocator) const
{
    RapidjsonValue json(rapidjson::kObjectType);
    TO_RAPIDJSON((*this), json, allocator, source_road)
    return json;
}
} // namespace nano_fmm
