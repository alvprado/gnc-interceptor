#include "visualization/visualization_node.hpp"

#include <array>
#include <rclcpp_components/register_node_macro.hpp>
#include <string>
#include <utility>

namespace gnc_ros
{
namespace
{

[[nodiscard]] visualization_msgs::msg::Marker makeSphereMarker(std::string const& ns,
                                                                std::array<float, 4> const& rgba)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world";
    marker.ns = ns;
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = marker.scale.y = marker.scale.z = 60.0;
    marker.color.r = rgba[0];
    marker.color.g = rgba[1];
    marker.color.b = rgba[2];
    marker.color.a = rgba[3];
    marker.pose.orientation.w = 1.0;
    return marker;
}

}  // namespace

VisualizationNode::VisualizationNode(rclcpp::NodeOptions const& options)
    : Node("visualization_node", options),
      target_marker_(makeSphereMarker("target", {1.0F, 0.2F, 0.2F, 1.0F})),
      interceptor_marker_(makeSphereMarker("interceptor", {0.2F, 0.6F, 1.0F, 1.0F}))
{
    auto const qos = rclcpp::QoS{10};
    marker_publisher_ =
        create_publisher<visualization_msgs::msg::MarkerArray>("visualization/markers", qos);
    target_subscription_ = create_subscription<gnc_interfaces::msg::TargetState>(
        "target/ground_truth", qos,
        [this](gnc_interfaces::msg::TargetState::ConstSharedPtr state)
        { targetCallback(std::move(state)); });
    interceptor_subscription_ = create_subscription<gnc_interfaces::msg::InterceptorState>(
        "interceptor/state", qos,
        [this](gnc_interfaces::msg::InterceptorState::ConstSharedPtr state)
        { interceptorCallback(std::move(state)); });
}

void VisualizationNode::targetCallback(gnc_interfaces::msg::TargetState::ConstSharedPtr state)
{
    target_marker_.header.stamp = state->header.stamp;
    target_marker_.pose.position = state->position_m;
    publishMarkers();
}

void VisualizationNode::interceptorCallback(
    gnc_interfaces::msg::InterceptorState::ConstSharedPtr state)
{
    interceptor_marker_.header.stamp = state->header.stamp;
    interceptor_marker_.pose.position = state->position_m;
    interceptor_marker_.pose.orientation = state->attitude;
    publishMarkers();
}

void VisualizationNode::publishMarkers()
{
    visualization_msgs::msg::MarkerArray markers;
    markers.markers = {target_marker_, interceptor_marker_};
    marker_publisher_->publish(markers);
}

}  // namespace gnc_ros

RCLCPP_COMPONENTS_REGISTER_NODE(gnc_ros::VisualizationNode)
