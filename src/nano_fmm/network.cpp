#include "nano_fmm/network.hpp"

namespace nano_fmm
{

std::shared_ptr<Config> Network::config()
{
    if (!config_) {
        config_ = std::make_shared<Config>();
    }
    return config_;
}

void Network::add(const Eigen::Ref<RowVectors> &polyline, int64_t id)
{
    //
}
void Network::add(const std::vector<RowVectors> &polylines,
                  std::optional<int64_t> ids)
{
    //
}

int Network::load(const std::string &path)
{
    //
}
bool Network::dump(const std::string &path) const
{
    //
}
} // namespace nano_fmm
