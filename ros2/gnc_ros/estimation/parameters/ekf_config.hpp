#pragma once

#include "estimation/extended_kalman_filter.hpp"

namespace rclcpp
{
class Node;
}

namespace gnc_ros
{

/// @brief Declare and read the EKF's startup-only ROS parameters.
[[nodiscard]] estimation::EKFTargetStateEstimationConfig readEkfConfig(rclcpp::Node& node);

/// @brief Construct the EKF from its configuration.
[[nodiscard]] estimation::EKFTargetStateEstimation makeEstimator(
    estimation::EKFTargetStateEstimationConfig const& config);

}  // namespace gnc_ros
