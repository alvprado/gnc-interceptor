#include "visualization/visualization_node.hpp"

#include <array>
#include <rclcpp_components/register_node_macro.hpp>
#include <string>
#include <utility>

#include "common/parameter_utils.hpp"

namespace gnc_ros
{
namespace
{

[[nodiscard]] visualization_msgs::msg::Marker makeMarker(std::string const& ns,
                                                         std::array<float, 4> const& rgba,
                                                         std::int32_t type, double scale)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world";
    marker.ns = ns;
    marker.id = 0;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = marker.scale.y = marker.scale.z = scale;
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
      target_marker_(makeMarker("target", {1.0F, 0.2F, 0.2F, 1.0F},
                                 visualization_msgs::msg::Marker::SPHERE,
                                 readParameter(*this, "target", "diameter_m", 10.0))),
      interceptor_marker_(makeMarker("interceptor", {0.2F, 0.6F, 1.0F, 1.0F},
                                      visualization_msgs::msg::Marker::SPHERE,
                                      readParameter(*this, "interceptor", "diameter_m", 10.0))),
      target_trail_marker_(makeMarker("target_trail", {1.0F, 0.2F, 0.2F, 0.65F},
                                       visualization_msgs::msg::Marker::LINE_STRIP,
                                       readParameter(*this, "trail", "width_m", 0.1))),
      interceptor_trail_marker_(makeMarker("interceptor_trail", {0.2F, 0.6F, 1.0F, 0.65F},
                                            visualization_msgs::msg::Marker::LINE_STRIP,
                                            target_trail_marker_.scale.x))
{
    history_points_ = static_cast<std::size_t>(
        readParameter<std::int64_t>(*this, "trail", "history_points", 300));
    sample_period_ns_ = rclcpp::Duration::from_seconds(
                            readParameter(*this, "trail", "sample_period_s", 0.1))
                            .nanoseconds();

    auto const qos = rclcpp::QoS{10};
    marker_publisher_ =
        create_publisher<visualization_msgs::msg::MarkerArray>("visualization/markers", qos);
    target_subscription_ = create_subscription<gnc_interfaces::msg::TargetState>(
        "target/ground_truth", qos, [this](gnc_interfaces::msg::TargetState::ConstSharedPtr state)
        { targetCallback(std::move(state)); });
    interceptor_subscription_ = create_subscription<gnc_interfaces::msg::InterceptorState>(
        "interceptor/state", qos,
        [this](gnc_interfaces::msg::InterceptorState::ConstSharedPtr state)
        { interceptorCallback(std::move(state)); });
}

void VisualizationNode::targetCallback(gnc_interfaces::msg::TargetState::ConstSharedPtr state)
{
    updateTrail(state->header, state->position_m, target_history_, target_trail_marker_);
    target_marker_.header = target_trail_marker_.header;
    target_marker_.pose.position = state->position_m;
    publishMarkers();
}

void VisualizationNode::interceptorCallback(
    gnc_interfaces::msg::InterceptorState::ConstSharedPtr state)
{
    updateTrail(state->header, state->position_m, interceptor_history_, interceptor_trail_marker_);
    interceptor_marker_.header = interceptor_trail_marker_.header;
    interceptor_marker_.pose.position = state->position_m;
    interceptor_marker_.pose.orientation = state->attitude;
    publishMarkers();
}

void VisualizationNode::updateTrail(std_msgs::msg::Header const& header,
                                    geometry_msgs::msg::Point const& position, Trail& history,
                                    visualization_msgs::msg::Marker& marker)
{
    auto const stamp_ns = rclcpp::Time(header.stamp).nanoseconds();
    auto const frame = header.frame_id.empty() ? std::string{"world"} : header.frame_id;
    if (stamp_ns < history.last_received_ns || frame != marker.header.frame_id)
    {
        history.points.clear();
    }
    marker.header = header;
    marker.header.frame_id = frame;
    history.last_received_ns = stamp_ns;
    if (history.points.empty() || stamp_ns - history.last_sample_ns >= sample_period_ns_)
    {
        history.points.push_back(position);
        history.last_sample_ns = stamp_ns;
        if (history.points.size() > history_points_)
        {
            history.points.pop_front();
        }
    }
    else if (stamp_ns == history.last_sample_ns)
    {
        history.points.back() = position;
    }

    marker.points.assign(history.points.begin(), history.points.end());
    // Keep the trail attached to the live marker between history samples.
    if (stamp_ns != history.last_sample_ns)
    {
        if (marker.points.size() == history_points_)
        {
            marker.points.erase(marker.points.begin());
        }
        marker.points.push_back(position);
    }
    marker.action = marker.points.size() >= 2 ? visualization_msgs::msg::Marker::ADD
                                              : visualization_msgs::msg::Marker::DELETE;
}

void VisualizationNode::publishMarkers()
{
    visualization_msgs::msg::MarkerArray markers;
    markers.markers.reserve(4);
    if (!target_history_.points.empty())
    {
        markers.markers.push_back(target_marker_);
        markers.markers.push_back(target_trail_marker_);
    }
    if (!interceptor_history_.points.empty())
    {
        markers.markers.push_back(interceptor_marker_);
        markers.markers.push_back(interceptor_trail_marker_);
    }
    marker_publisher_->publish(markers);
}

}  // namespace gnc_ros

RCLCPP_COMPONENTS_REGISTER_NODE(gnc_ros::VisualizationNode)
