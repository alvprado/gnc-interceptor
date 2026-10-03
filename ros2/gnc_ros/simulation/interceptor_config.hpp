#pragma once

#include "simulation/uav_3dof_model.hpp"

namespace rclcpp
{
class Node;
}

namespace gnc_ros
{

/// @brief Interceptor model construction parameters: physical constants and flight envelope limits.
struct InterceptorConfig
{
    simulation::UAV3DofModelParams params{};
    simulation::UAV3DofModelLimits limits{};
};

/// @brief Declare and read the interceptor model's startup-only ROS parameters.
[[nodiscard]] InterceptorConfig readInterceptorConfig(rclcpp::Node& node);

/// @brief Construct the interceptor model from its configuration.
[[nodiscard]] simulation::UAV3DofModel makeInterceptorModel(InterceptorConfig const& config);

}  // namespace gnc_ros
