#include "gnc_ros/guidance_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <utility>

namespace gnc_ros
{

GuidanceNode::GuidanceNode(rclcpp::NodeOptions const& options) : Node("guidance_node", options)
{
    auto const qos = rclcpp::QoS{10};
    vehicle_subscription_ = create_subscription<gnc_interfaces::msg::VehicleState>(
        "vehicle/state", qos, [this](gnc_interfaces::msg::VehicleState::ConstSharedPtr state)
        { latest_vehicle_state_ = std::move(state); });
    estimate_subscription_ = create_subscription<gnc_interfaces::msg::TargetEstimate>(
        "target/estimate", qos, [this](gnc_interfaces::msg::TargetEstimate::ConstSharedPtr estimate)
        { latest_estimate_ = std::move(estimate); });
    command_publisher_ =
        create_publisher<gnc_interfaces::msg::GuidanceCommand>("guidance/command", qos);

    // TODO: Connect controller evaluation once input timing and validity are handled.
    RCLCPP_INFO(get_logger(), "Guidance skeleton ready; the controller is not connected yet.");
}

}  // namespace gnc_ros

RCLCPP_COMPONENTS_REGISTER_NODE(gnc_ros::GuidanceNode)
