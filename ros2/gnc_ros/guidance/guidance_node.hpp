#pragma once

#include <rclcpp/rclcpp.hpp>

#include "gnc_interfaces/msg/guidance_command.hpp"
#include "gnc_interfaces/msg/interceptor_state.hpp"
#include "gnc_interfaces/msg/target_estimate.hpp"
#include "guidance/parameters/controller_config.hpp"
#include "guidance/parameters/guidance_config.hpp"

namespace gnc_ros
{

/// @brief ROS interfaces for the guidance controller.
class GuidanceNode : public rclcpp::Node
{
public:
    explicit GuidanceNode(rclcpp::NodeOptions const& options = rclcpp::NodeOptions{});

private:
    // Callbacks
    void interceptorStateCallback(gnc_interfaces::msg::InterceptorState::ConstSharedPtr state);
    void targetEstimateCallback(gnc_interfaces::msg::TargetEstimate::ConstSharedPtr estimate);
    void guidanceCallback();

    // Subscribers
    rclcpp::Subscription<gnc_interfaces::msg::InterceptorState>::SharedPtr interceptor_subscription_;
    rclcpp::Subscription<gnc_interfaces::msg::TargetEstimate>::SharedPtr estimate_subscription_;

    // Publisher
    rclcpp::Publisher<gnc_interfaces::msg::GuidanceCommand>::SharedPtr command_publisher_;

    // Timer
    rclcpp::TimerBase::SharedPtr guidance_timer_;

    // Components
    GuidanceConfig guidance_config_;
    GuidanceController controller_;

    // State
    gnc_interfaces::msg::InterceptorState::ConstSharedPtr interceptor_state_;
    gnc_interfaces::msg::TargetEstimate::ConstSharedPtr target_estimate_state_;
};

}  // namespace gnc_ros
