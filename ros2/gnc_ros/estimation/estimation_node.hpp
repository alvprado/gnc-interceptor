#pragma once

#include <message_filters/subscriber.hpp>
#include <message_filters/time_synchronizer.hpp>
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
    void measurementCallback(gnc_interfaces::msg::InterceptorState::ConstSharedPtr state,
                             gnc_interfaces::msg::RadarMeasurement::ConstSharedPtr measurement);
    void estimationCallback();

    // Subscribers
    message_filters::Subscriber<gnc_interfaces::msg::InterceptorState> interceptor_subscription_;
    message_filters::Subscriber<gnc_interfaces::msg::RadarMeasurement> radar_subscription_;
    message_filters::TimeSynchronizer<gnc_interfaces::msg::InterceptorState,
                                      gnc_interfaces::msg::RadarMeasurement> measurement_sync_{10};

    // Publisher
    rclcpp::Publisher<gnc_interfaces::msg::TargetEstimate>::SharedPtr estimate_publisher_;

    // Timer
    rclcpp::TimerBase::SharedPtr estimation_timer_;

    // Components
    EstimationConfig estimation_config_;
    estimation::EKFTargetStateEstimation ekf_;
};

}  // namespace gnc_ros
