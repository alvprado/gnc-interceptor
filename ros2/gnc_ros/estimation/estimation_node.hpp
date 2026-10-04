#pragma once

#include <rclcpp/rclcpp.hpp>

#include "estimation/extended_kalman_filter.hpp"
#include "estimation/parameters/ekf_config.hpp"
#include "estimation/parameters/estimation_config.hpp"
#include "gnc_interfaces/msg/interceptor_state.hpp"
#include "gnc_interfaces/msg/radar_measurement.hpp"
#include "gnc_interfaces/msg/target_estimate.hpp"

namespace gnc_ros
{

/// @brief ROS interfaces for target state estimation.
class EstimationNode : public rclcpp::Node
{
public:
    explicit EstimationNode(rclcpp::NodeOptions const& options = rclcpp::NodeOptions{});

private:
    // Callbacks
    void interceptorStateCallback(gnc_interfaces::msg::InterceptorState::ConstSharedPtr state);
    void radarMeasurementCallback(gnc_interfaces::msg::RadarMeasurement::ConstSharedPtr measurement);
    void estimationCallback();

    // Subscribers
    rclcpp::Subscription<gnc_interfaces::msg::InterceptorState>::SharedPtr interceptor_subscription_;
    rclcpp::Subscription<gnc_interfaces::msg::RadarMeasurement>::SharedPtr radar_subscription_;

    // Publisher
    rclcpp::Publisher<gnc_interfaces::msg::TargetEstimate>::SharedPtr estimate_publisher_;

    // Timer
    rclcpp::TimerBase::SharedPtr estimation_timer_;

    // Components
    EstimationConfig estimation_config_;
    estimation::EKFTargetStateEstimation ekf_;

    // State
    gnc_interfaces::msg::InterceptorState::ConstSharedPtr interceptor_state_;
};

}  // namespace gnc_ros
