#include "gnc_ros/simulation_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <utility>

namespace gnc_ros
{

SimulationNode::SimulationNode(rclcpp::NodeOptions const& options)
    : Node("simulation_node", options)
{
    auto const qos = rclcpp::QoS{10};
    interceptor_publisher_ =
        create_publisher<gnc_interfaces::msg::InterceptorState>("interceptor/state", qos);
    target_publisher_ =
        create_publisher<gnc_interfaces::msg::TargetState>("target/ground_truth", qos);
    radar_publisher_ =
        create_publisher<gnc_interfaces::msg::RadarMeasurement>("radar/measurement", qos);
    clock_publisher_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS{});
    command_subscription_ = create_subscription<gnc_interfaces::msg::GuidanceCommand>(
        "guidance/command", qos,
        [this](gnc_interfaces::msg::GuidanceCommand::ConstSharedPtr command)
        { latest_command_ = std::move(command); });
    reset_service_ = create_service<gnc_interfaces::srv::ResetSimulation>(
        "simulation/reset",
        [](gnc_interfaces::srv::ResetSimulation::Request::SharedPtr,
           gnc_interfaces::srv::ResetSimulation::Response::SharedPtr response)
        {
            response->success = false;
            response->message = "Reset is not implemented in the simulation skeleton.";
        });

    // TODO: Connect model stepping and publish each sample with its simulation timestamp.
    RCLCPP_INFO(get_logger(), "Simulation skeleton ready; model stepping is not connected yet.");
}

}  // namespace gnc_ros

RCLCPP_COMPONENTS_REGISTER_NODE(gnc_ros::SimulationNode)
