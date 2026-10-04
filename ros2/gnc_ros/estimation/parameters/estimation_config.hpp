#pragma once

namespace rclcpp
{
class Node;
}

namespace gnc_ros
{

/// @brief Target estimate publish loop period.
struct EstimationConfig
{
    double dt_s{0.1};
};

/// @brief Declare and read the estimation loop's startup-only ROS parameters.
[[nodiscard]] EstimationConfig readEstimationConfig(rclcpp::Node& node);

}  // namespace gnc_ros
