#pragma once

#include <rclcpp/rclcpp.hpp>

#include "gnc_interfaces/msg/guidance_command.hpp"
#include "gnc_interfaces/msg/target_estimate.hpp"
#include "gnc_interfaces/msg/vehicle_state.hpp"

namespace gnc_ros
{

/// @brief ROS interfaces for the guidance controller.
class GuidanceNode : public rclcpp::Node
{
public:
    explicit GuidanceNode(rclcpp::NodeOptions const& options = rclcpp::NodeOptions{});

private:
    rclcpp::Subscription<gnc_interfaces::msg::VehicleState>::SharedPtr vehicle_subscription_;
    rclcpp::Subscription<gnc_interfaces::msg::TargetEstimate>::SharedPtr estimate_subscription_;
    rclcpp::Publisher<gnc_interfaces::msg::GuidanceCommand>::SharedPtr command_publisher_;
    gnc_interfaces::msg::VehicleState::ConstSharedPtr latest_vehicle_state_;
    gnc_interfaces::msg::TargetEstimate::ConstSharedPtr latest_estimate_;
};

}  // namespace gnc_ros
