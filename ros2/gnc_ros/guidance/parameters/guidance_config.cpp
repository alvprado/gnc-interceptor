#include "guidance/parameters/guidance_config.hpp"

#include "common/parameter_utils.hpp"

namespace gnc_ros
{
namespace
{

constexpr auto prefix = "guidance";

}  // namespace

GuidanceConfig readGuidanceConfig(rclcpp::Node& node)
{
    GuidanceConfig config;
    config.dt_s = readParameter(node, prefix, "dt_s", config.dt_s);
    return config;
}

}  // namespace gnc_ros
