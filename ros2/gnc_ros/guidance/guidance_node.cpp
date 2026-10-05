#include "guidance/guidance_node.hpp"

#include <rclcpp/create_timer.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <utility>

#include "common/converters.hpp"

namespace gnc_ros
{

GuidanceNode::GuidanceNode(rclcpp::NodeOptions const& options)
    : Node("guidance_node", options),
      guidance_config_(readGuidanceConfig(*this)),
      controller_(makeGuidanceController(readControllerConfig(*this)))
{
    auto const qos = rclcpp::QoS{10};
    interceptor_subscription_ = create_subscription<gnc_interfaces::msg::InterceptorState>(
        "interceptor/state", qos,
        [this](gnc_interfaces::msg::InterceptorState::ConstSharedPtr state)
        { interceptorStateCallback(std::move(state)); });
    estimate_subscription_ = create_subscription<gnc_interfaces::msg::TargetEstimate>(
        "target/estimate", qos,
        [this](gnc_interfaces::msg::TargetEstimate::ConstSharedPtr estimate)
        { targetEstimateCallback(std::move(estimate)); });
    command_publisher_ =
        create_publisher<gnc_interfaces::msg::GuidanceCommand>("guidance/command", qos);

    guidance_timer_ = rclcpp::create_timer(this, get_clock(),
                                           rclcpp::Duration::from_seconds(guidance_config_.dt_s),
                                           [this] { guidanceCallback(); });
}

void GuidanceNode::interceptorStateCallback(
    gnc_interfaces::msg::InterceptorState::ConstSharedPtr state)
{
    interceptor_state_ = std::move(state);
}

void GuidanceNode::targetEstimateCallback(
    gnc_interfaces::msg::TargetEstimate::ConstSharedPtr estimate)
{
    target_estimate_state_ = std::move(estimate);
}

math::CartesianState GuidanceNode::targetStateAtInterceptorTime() const
{
    auto target = fromMsg(target_estimate_state_->state);
    double const target_age_s = (fromMsg(interceptor_state_->header.stamp) -
                                  fromMsg(target_estimate_state_->state.header.stamp)).count();
    target.position_m += target_age_s * target.velocity_mps +
                         0.5 * target_age_s * target_age_s * target.acceleration_mps2;
    target.velocity_mps += target_age_s * target.acceleration_mps2;
    return target;
}

void GuidanceNode::guidanceCallback()
{
    bool const estimate_valid = target_estimate_state_ && target_estimate_state_->status ==
                                                               gnc_interfaces::msg::TargetEstimate::VALID;
    if (!interceptor_state_ || !estimate_valid)
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                            "Waiting for interceptor state and a valid target estimate.");
        return;
    }

    auto const target = targetStateAtInterceptorTime();
    auto const interceptor = fromMsg(*interceptor_state_).cartesian;
    auto const control = controller_.step(target, interceptor, guidance_config_.dt_s);

    command_publisher_->publish(toMsg(control, now(), "interceptor_body"));
}

}  // namespace gnc_ros

RCLCPP_COMPONENTS_REGISTER_NODE(gnc_ros::GuidanceNode)
