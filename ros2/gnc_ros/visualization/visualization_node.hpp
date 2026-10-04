#pragma once

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "gnc_interfaces/msg/interceptor_state.hpp"
#include "gnc_interfaces/msg/target_state.hpp"

namespace gnc_ros
{

/// @brief Republishes target and interceptor ground-truth states as RViz markers.
class VisualizationNode : public rclcpp::Node
{
public:
    explicit VisualizationNode(rclcpp::NodeOptions const& options = rclcpp::NodeOptions{});

private:
    void targetCallback(gnc_interfaces::msg::TargetState::ConstSharedPtr state);
    void interceptorCallback(gnc_interfaces::msg::InterceptorState::ConstSharedPtr state);
    void publishMarkers();

    rclcpp::Subscription<gnc_interfaces::msg::TargetState>::SharedPtr target_subscription_;
    rclcpp::Subscription<gnc_interfaces::msg::InterceptorState>::SharedPtr interceptor_subscription_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;

    visualization_msgs::msg::Marker target_marker_;
    visualization_msgs::msg::Marker interceptor_marker_;
};

}  // namespace gnc_ros
