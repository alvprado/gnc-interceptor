#include "estimation/estimation_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <utility>

namespace gnc_ros
{

EstimationNode::EstimationNode(rclcpp::NodeOptions const& options)
    : Node("estimation_node", options)
{
    auto const qos = rclcpp::QoS{10};
    interceptor_subscription_ = create_subscription<gnc_interfaces::msg::InterceptorState>(
        "interceptor/state", qos,
        [this](gnc_interfaces::msg::InterceptorState::ConstSharedPtr state)
        { latest_interceptor_state_ = std::move(state); });
    radar_subscription_ = create_subscription<gnc_interfaces::msg::RadarMeasurement>(
        "radar/measurement", qos,
        [this](gnc_interfaces::msg::RadarMeasurement::ConstSharedPtr measurement)
        { latest_measurement_ = std::move(measurement); });
    estimate_publisher_ =
        create_publisher<gnc_interfaces::msg::TargetEstimate>("target/estimate", qos);

    // TODO: Match interceptor state to measurement time before calling the EKF.
    RCLCPP_INFO(get_logger(), "Estimation skeleton ready; the EKF is not connected yet.");
}

}  // namespace gnc_ros

RCLCPP_COMPONENTS_REGISTER_NODE(gnc_ros::EstimationNode)
