#include "estimation/estimation_node.hpp"

#include <array>
#include <cstdint>
#include <rclcpp/create_timer.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <utility>

#include "common/converters.hpp"

namespace gnc_ros
{
namespace
{

static_assert(static_cast<std::uint8_t>(estimation::EKFStatus::Valid) ==
              gnc_interfaces::msg::TargetEstimate::VALID);
static_assert(static_cast<std::uint8_t>(estimation::EKFStatus::StaleTimestamp) ==
              gnc_interfaces::msg::TargetEstimate::STALE_TIMESTAMP);
static_assert(static_cast<std::uint8_t>(estimation::EKFStatus::SingularInnovationCovariance) ==
              gnc_interfaces::msg::TargetEstimate::SINGULAR_INNOVATION_COVARIANCE);

void writeCovariance(estimation::StateCov const& covariance, std::array<double, 81>& out)
{
    for (Eigen::Index row = 0; row < covariance.rows(); ++row)
    {
        for (Eigen::Index col = 0; col < covariance.cols(); ++col)
        {
            out[static_cast<std::size_t>(row * covariance.cols() + col)] = covariance(row, col);
        }
    }
}

}  // namespace

EstimationNode::EstimationNode(rclcpp::NodeOptions const& options)
    : Node("estimation_node", options),
      estimation_config_(readEstimationConfig(*this)),
      ekf_(makeEstimator(readEkfConfig(*this)))
{
    auto const qos = rclcpp::QoS{10};
    interceptor_subscription_ = create_subscription<gnc_interfaces::msg::InterceptorState>(
        "interceptor/state", qos,
        [this](gnc_interfaces::msg::InterceptorState::ConstSharedPtr state)
        { interceptorStateCallback(std::move(state)); });
    radar_subscription_ = create_subscription<gnc_interfaces::msg::RadarMeasurement>(
        "radar/measurement", qos,
        [this](gnc_interfaces::msg::RadarMeasurement::ConstSharedPtr measurement)
        { radarMeasurementCallback(std::move(measurement)); });
    estimate_publisher_ =
        create_publisher<gnc_interfaces::msg::TargetEstimate>("target/estimate", qos);

    estimation_timer_ = rclcpp::create_timer(this, get_clock(),
                                             rclcpp::Duration::from_seconds(estimation_config_.dt_s),
                                             [this] { estimationCallback(); });
}

void EstimationNode::interceptorStateCallback(
    gnc_interfaces::msg::InterceptorState::ConstSharedPtr state)
{
    interceptor_state_ = std::move(state);
}

void EstimationNode::radarMeasurementCallback(
    gnc_interfaces::msg::RadarMeasurement::ConstSharedPtr measurement)
{
    if (!interceptor_state_)
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                            "Dropping radar measurement: no interceptor state yet.");
        return;
    }
    ekf_.processMeasurement(fromMsg(*measurement), fromMsg(*interceptor_state_));
}

void EstimationNode::estimationCallback()
{
    if (!ekf_.isInitialized())
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                            "Waiting for the first radar measurement.");
        return;
    }

    auto const stamp = now();
    math::Timestamp const query_time = fromMsg(stamp);
    auto const output = ekf_.stateEstimateAt(query_time);

    gnc_interfaces::msg::TargetEstimate estimate;
    estimate.state = toMsg(output.target_state_estimate, stamp, "world");
    writeCovariance(ekf_.errorCovarianceAt(query_time), estimate.covariance);
    estimate.status = static_cast<std::uint8_t>(output.filter_status);

    estimate_publisher_->publish(estimate);
}

}  // namespace gnc_ros

RCLCPP_COMPONENTS_REGISTER_NODE(gnc_ros::EstimationNode)
