#pragma once

#include <cstdint>
#include <deque>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "gnc_interfaces/msg/interceptor_state.hpp"
#include "gnc_interfaces/msg/target_state.hpp"

namespace gnc_ros
{

/// @brief Displays ground-truth poses and bounded trajectory trails in RViz.
class VisualizationNode : public rclcpp::Node
{
public:
    explicit VisualizationNode(rclcpp::NodeOptions const& options = rclcpp::NodeOptions{});

private:
    struct Trail
    {
        std::deque<geometry_msgs::msg::Point> points;
        std::int64_t last_received_ns{};
        std::int64_t last_sample_ns{};
    };

    void targetCallback(gnc_interfaces::msg::TargetState::ConstSharedPtr state);
    void interceptorCallback(gnc_interfaces::msg::InterceptorState::ConstSharedPtr state);
    void publishMarkers();
    void updateTrail(std_msgs::msg::Header const& header, geometry_msgs::msg::Point const& position,
                     Trail& history, visualization_msgs::msg::Marker& marker);

    rclcpp::Subscription<gnc_interfaces::msg::TargetState>::SharedPtr target_subscription_;
    rclcpp::Subscription<gnc_interfaces::msg::InterceptorState>::SharedPtr
        interceptor_subscription_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;

    visualization_msgs::msg::Marker target_marker_;
    visualization_msgs::msg::Marker interceptor_marker_;
    visualization_msgs::msg::Marker target_trail_marker_;
    visualization_msgs::msg::Marker interceptor_trail_marker_;
    Trail target_history_;
    Trail interceptor_history_;
    std::size_t history_points_{};
    std::int64_t sample_period_ns_{};
};

}  // namespace gnc_ros
