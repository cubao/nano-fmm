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

Eigen::Vector3d Network::cheap_ruler_k(double latitude)
{
    // lookup table
#ifdef K
#undef K
#endif
#define K(lat) utils::cheap_ruler_k((double)lat)
    Eigen::Vector3d Ks[] = {
        // clang-format off
        K(0),K(1),K(2),K(3),K(4),K(5),K(6),K(7),K(8),K(9),K(10),K(11),K(12),K(13),
        K(14),K(15),K(16),K(17),K(18),K(19),K(20),K(21),K(22),K(23),K(24),K(25),K(26),
        K(27),K(28),K(29),K(30),K(31),K(32),K(33),K(34),K(35),K(36),K(37),K(38),K(39),
        K(40),K(41),K(42),K(43),K(44),K(45),K(46),K(47),K(48),K(49),K(50),K(51),K(52),
        K(53),K(54),K(55),K(56),K(57),K(58),K(59),K(60),K(61),K(62),K(63),K(64),K(65),
        K(66),K(67),K(68),K(69),K(70),K(71),K(72),K(73),K(74),K(75),K(76),K(77),K(78),
        K(79),K(80),K(81),K(82),K(83),K(84),K(85),K(86),K(87),K(88),K(89),K(90)
        // clang-format on
    };
    int idx =
        std::min(90, static_cast<int>(std::floor(std::fabs(latitude) + 0.5)));
    return Ks[idx];
}

} // namespace nano_fmm
