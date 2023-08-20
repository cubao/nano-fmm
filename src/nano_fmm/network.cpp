#include "nano_fmm/network.hpp"
#include "nano_fmm/utils.hpp"

namespace nano_fmm
{

std::shared_ptr<Config> Network::config()
{
    if (!config_) {
        config_ = std::make_shared<Config>();
    }
    return config_;
}

void Network::config(std::shared_ptr<Config> config) { config_ = config; }

void Network::add_node(int64_t node_id, const Eigen::Ref<RowVectors> &polyline)
{
    //
}
void Network::add_edge(int64_t edge_id, int64_t source_node,
                       int64_t target_node)
{
    //
}

int Network::load(const std::string &path)
{
    //
    return 0;
}
bool Network::dump(const std::string &path) const
{
    //
    return false;
}

} // namespace nano_fmm
