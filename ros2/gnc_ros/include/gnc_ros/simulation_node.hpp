#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include "gnc_interfaces/msg/guidance_command.hpp"
#include "gnc_interfaces/msg/radar_measurement.hpp"
#include "gnc_interfaces/msg/target_state.hpp"
#include "gnc_interfaces/msg/interceptor_state.hpp"
#include "gnc_interfaces/srv/reset_simulation.hpp"

namespace gnc_ros
{

/// @brief ROS interfaces for the simulated interceptor, target, and radar.
class SimulationNode : public rclcpp::Node
{
public:
    explicit SimulationNode(rclcpp::NodeOptions const& options = rclcpp::NodeOptions{});

private:
    rclcpp::Publisher<gnc_interfaces::msg::InterceptorState>::SharedPtr interceptor_publisher_;
    rclcpp::Publisher<gnc_interfaces::msg::TargetState>::SharedPtr target_publisher_;
    rclcpp::Publisher<gnc_interfaces::msg::RadarMeasurement>::SharedPtr radar_publisher_;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;
    rclcpp::Subscription<gnc_interfaces::msg::GuidanceCommand>::SharedPtr command_subscription_;
    rclcpp::Service<gnc_interfaces::srv::ResetSimulation>::SharedPtr reset_service_;
    gnc_interfaces::msg::GuidanceCommand::ConstSharedPtr latest_command_;
};

}  // namespace gnc_ros
