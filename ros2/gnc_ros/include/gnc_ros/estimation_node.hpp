#pragma once

#include <rclcpp/rclcpp.hpp>

#include "gnc_interfaces/msg/radar_measurement.hpp"
#include "gnc_interfaces/msg/target_estimate.hpp"
#include "gnc_interfaces/msg/vehicle_state.hpp"

namespace gnc_ros
{

/// @brief ROS interfaces for target state estimation.
class EstimationNode : public rclcpp::Node
{
public:
    explicit EstimationNode(rclcpp::NodeOptions const& options = rclcpp::NodeOptions{});

private:
    rclcpp::Subscription<gnc_interfaces::msg::VehicleState>::SharedPtr vehicle_subscription_;
    rclcpp::Subscription<gnc_interfaces::msg::RadarMeasurement>::SharedPtr radar_subscription_;
    rclcpp::Publisher<gnc_interfaces::msg::TargetEstimate>::SharedPtr estimate_publisher_;
    gnc_interfaces::msg::VehicleState::ConstSharedPtr latest_vehicle_state_;
    gnc_interfaces::msg::RadarMeasurement::ConstSharedPtr latest_measurement_;
};

}  // namespace gnc_ros
