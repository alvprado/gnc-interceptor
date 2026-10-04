#pragma once

namespace rclcpp
{
class Node;
}

namespace gnc_ros
{

/// @brief Guidance loop update period.
struct GuidanceConfig
{
    double dt_s{0.05};
};

/// @brief Declare and read the guidance loop's startup-only ROS parameters.
[[nodiscard]] GuidanceConfig readGuidanceConfig(rclcpp::Node& node);

}  // namespace gnc_ros
