#include "estimation/parameters/estimation_config.hpp"

#include "common/parameter_utils.hpp"

namespace gnc_ros
{
namespace
{

constexpr auto prefix = "estimation";

}  // namespace

EstimationConfig readEstimationConfig(rclcpp::Node& node)
{
    EstimationConfig config;
    config.dt_s = readParameter(node, prefix, "dt_s", config.dt_s);
    return config;
}

}  // namespace gnc_ros
