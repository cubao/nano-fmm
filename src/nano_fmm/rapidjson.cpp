#include "nano_fmm/rapidjson_helpers.hpp"
#include "nano_fmm/network/ubodt.hpp"

namespace nano_fmm
{
UbodtRecord &UbodtRecord::from_rapidjson(const RapidjsonValue &json)
{
    return *this;
}
RapidjsonValue UbodtRecord::to_rapidjson(RapidjsonAllocator &allocator) const
{
    RapidjsonValue json(rapidjson::kObjectType);
    return json;
}
} // namespace nano_fmm
