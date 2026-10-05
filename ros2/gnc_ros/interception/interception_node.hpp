#pragma once

#include <chrono>
#include <message_filters/subscriber.hpp>
#include <message_filters/time_synchronizer.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "gnc_interfaces/action/intercept.hpp"
#include "gnc_interfaces/msg/interceptor_state.hpp"
#include "gnc_interfaces/msg/target_estimate.hpp"
#include "gnc_interfaces/msg/target_state.hpp"
#include "gnc_interfaces/srv/simulation_control.hpp"

namespace gnc_ros
{

/// @brief Coordinate one interception attempt and evaluate synchronized ground truth.
class InterceptionNode : public rclcpp::Node
{
public:
    explicit InterceptionNode(rclcpp::NodeOptions const& options = rclcpp::NodeOptions{});

private:
    using Intercept = gnc_interfaces::action::Intercept;
    using GoalHandle = rclcpp_action::ServerGoalHandle<Intercept>;
    using Control = gnc_interfaces::srv::SimulationControl;
    enum class Phase
    {
        Idle,
        Preparing,
        WaitingEstimate,
        Starting,
        Running,
        Finishing
    };

    rclcpp_action::GoalResponse goalCallback(std::shared_ptr<Intercept::Goal const> goal);
    void acceptedCallback(std::shared_ptr<GoalHandle> goal);
    void stateCallback(gnc_interfaces::msg::InterceptorState::ConstSharedPtr interceptor,
                       gnc_interfaces::msg::TargetState::ConstSharedPtr target);
    void tick();
    void requestControl(std::uint8_t command);
    void evaluate();
    void finish(bool success, std::string const& message);
    void complete();

    message_filters::Subscriber<gnc_interfaces::msg::InterceptorState> interceptor_subscription_;
    message_filters::Subscriber<gnc_interfaces::msg::TargetState> target_subscription_;
    message_filters::TimeSynchronizer<gnc_interfaces::msg::InterceptorState,
                                      gnc_interfaces::msg::TargetState>
        state_sync_{10};
    rclcpp::Subscription<gnc_interfaces::msg::TargetEstimate>::SharedPtr estimate_subscription_;
    rclcpp_action::Server<Intercept>::SharedPtr action_;
    rclcpp::Client<Control>::SharedPtr control_client_;
    std::optional<rclcpp::Client<Control>::FutureAndRequestId> control_future_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<GoalHandle> goal_;
    Phase phase_{Phase::Idle};
    bool goal_pending_{false};
    std::uint64_t run_id_{0};
    builtin_interfaces::msg::Time start_stamp_;
    std::chrono::steady_clock::time_point deadline_;
    gnc_interfaces::msg::TargetEstimate::ConstSharedPtr estimate_;
    gnc_interfaces::msg::InterceptorState::ConstSharedPtr interceptor_;
    gnc_interfaces::msg::TargetState::ConstSharedPtr target_;
    Intercept::Result result_;
    double next_feedback_s_{0.0};
};

}  // namespace gnc_ros
