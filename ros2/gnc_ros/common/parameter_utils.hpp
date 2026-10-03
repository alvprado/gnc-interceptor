#pragma once

#include <Eigen/Dense>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace gnc_ros
{

/// @brief Declare and read a startup-only (read-only) ROS parameter "<prefix>.<name>".
/// @tparam T The parameter's value type.
/// @param[in] node The node to declare the parameter on.
/// @param[in] prefix The parameter group, e.g. "model", "sensor" or "target".
/// @param[in] name The parameter's name within that group.
/// @param[in] default_value The value used if the parameter isn't set.
/// @returns The declared parameter's value.
template <typename T>
[[nodiscard]] T readParameter(rclcpp::Node& node, std::string const& prefix,
                              std::string const& name, T const& default_value)
{
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return node.declare_parameter<T>(prefix + "." + name, default_value, descriptor);
}

/// @brief Declare and read a startup-only "<prefix>.<name>" ROS parameter as a 3-vector.
/// @param[in] node The node to declare the parameter on.
/// @param[in] prefix The parameter group, e.g. "model", "sensor" or "target".
/// @param[in] name The parameter's name within that group.
/// @param[in] default_value The value used if the parameter isn't set.
/// @returns The declared parameter's value.
[[nodiscard]] inline Eigen::Vector3d readVectorParameter(rclcpp::Node& node,
                                                         std::string const& prefix,
                                                         std::string const& name,
                                                         Eigen::Vector3d const& default_value)
{
    auto const values = readParameter(node, prefix, name,
                                      std::vector<double>{default_value.x(), default_value.y(),
                                                          default_value.z()});
    return {values[0], values[1], values[2]};
}

}  // namespace gnc_ros
