#include "estimation/estimation_node.hpp"

#include <array>
#include <cstdint>
#include <rclcpp/create_timer.hpp>
#include <rclcpp_components/register_node_macro.hpp>

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
      ekf_config_(readEkfConfig(*this)),
      ekf_(makeEstimator(ekf_config_))
{
    auto const qos = rclcpp::QoS{10};
    interceptor_subscription_.subscribe(this, "interceptor/state", qos.get_rmw_qos_profile());
    radar_subscription_.subscribe(this, "radar/measurement", qos.get_rmw_qos_profile());
    measurement_sync_.connectInput(interceptor_subscription_, radar_subscription_);
    measurement_sync_.registerCallback(&EstimationNode::measurementCallback, this);
    estimate_publisher_ =
        create_publisher<gnc_interfaces::msg::TargetEstimate>("target/estimate", qos);

    estimation_timer_ = rclcpp::create_timer(
        this, get_clock(), rclcpp::Duration::from_seconds(estimation_config_.dt_s),
        [this] { estimationCallback(); });
}

void EstimationNode::measurementCallback(
    gnc_interfaces::msg::InterceptorState::ConstSharedPtr state,
    gnc_interfaces::msg::RadarMeasurement::ConstSharedPtr measurement)
{
    if (state->run_id != measurement->run_id || measurement->run_id < run_id_) return;
    if (measurement->run_id != run_id_)
    {
        run_id_ = measurement->run_id;
        ekf_ = makeEstimator(ekf_config_);
    }
    ekf_.processMeasurement(fromMsg(*measurement), fromMsg(*state));
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
    estimate.state.run_id = run_id_;
    writeCovariance(ekf_.errorCovarianceAt(query_time), estimate.covariance);
    estimate.status = static_cast<std::uint8_t>(output.filter_status);

    estimate_publisher_->publish(estimate);
}

}  // namespace gnc_ros

RCLCPP_COMPONENTS_REGISTER_NODE(gnc_ros::EstimationNode)
