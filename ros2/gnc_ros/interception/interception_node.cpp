#include "interception/interception_node.hpp"

#include <cmath>
#include <rclcpp_components/register_node_macro.hpp>

#include "common/converters.hpp"

namespace gnc_ros
{

InterceptionNode::InterceptionNode(rclcpp::NodeOptions const& options)
    : Node("interception_node", options)
{
    auto const qos = rclcpp::QoS{10};
    interceptor_subscription_.subscribe(this, "interceptor/state", qos.get_rmw_qos_profile());
    target_subscription_.subscribe(this, "target/ground_truth", qos.get_rmw_qos_profile());
    state_sync_.connectInput(interceptor_subscription_, target_subscription_);
    state_sync_.registerCallback(&InterceptionNode::stateCallback, this);
    estimate_subscription_ = create_subscription<gnc_interfaces::msg::TargetEstimate>(
        "target/estimate", qos,
        [this](gnc_interfaces::msg::TargetEstimate::ConstSharedPtr estimate)
        {
            if (!estimate_ || estimate->state.run_id > estimate_->state.run_id ||
                (estimate->state.run_id == estimate_->state.run_id &&
                 fromMsg(estimate->state.header.stamp) >= fromMsg(estimate_->state.header.stamp)))
                estimate_ = std::move(estimate);
        });
    control_client_ = create_client<Control>("simulation/control");
    action_ = rclcpp_action::create_server<Intercept>(
        this, "interception/start",
        [this](auto const&, auto goal) { return goalCallback(std::move(goal)); },
        [this](auto goal)
        {
            return goal == goal_ ? rclcpp_action::CancelResponse::ACCEPT
                                 : rclcpp_action::CancelResponse::REJECT;
        },
        [this](auto goal) { acceptedCallback(std::move(goal)); });
    timer_ = create_wall_timer(std::chrono::milliseconds{10}, [this] { tick(); });
}

rclcpp_action::GoalResponse InterceptionNode::goalCallback(
    std::shared_ptr<Intercept::Goal const> goal)
{
    if (goal_ || goal_pending_ || !control_client_->service_is_ready() ||
        !std::isfinite(goal->interception_distance_m) || goal->interception_distance_m <= 0.0 ||
        !std::isfinite(goal->max_interception_time_s) || goal->max_interception_time_s <= 0.0)
        return rclcpp_action::GoalResponse::REJECT;
    goal_pending_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void InterceptionNode::acceptedCallback(std::shared_ptr<GoalHandle> goal)
{
    goal_ = std::move(goal);
    goal_pending_ = false;
    result_ = Intercept::Result{};
    next_feedback_s_ = 0.0;
    estimate_.reset();
    phase_ = Phase::Preparing;
    requestControl(Control::Request::RESET);
}

void InterceptionNode::requestControl(std::uint8_t command)
{
    auto request = std::make_shared<Control::Request>();
    request->command = command;
    request->expected_run_id = run_id_;
    if (command == Control::Request::START)
        request->target_position_m = estimate_->state.position_m;
    control_future_.emplace(control_client_->async_send_request(request));
    deadline_ = std::chrono::steady_clock::now() + std::chrono::seconds{5};
}

void InterceptionNode::tick()
{
    if (!goal_) return;
    if (control_future_)
    {
        if (control_future_->wait_for(std::chrono::seconds{0}) != std::future_status::ready)
        {
            if (std::chrono::steady_clock::now() >= deadline_)
            {
                control_client_->remove_pending_request(*control_future_);
                control_future_.reset();
                if (phase_ != Phase::Finishing)
                    finish(false, "Simulation control timed out.");
                else
                {
                    result_.success = false;
                    result_.message += " Reset timed out; simulation state is unknown.";
                    complete();
                }
            }
            return;
        }
        auto response = control_future_->get();
        control_future_.reset();
        if (phase_ == Phase::Finishing)
        {
            if (!response->success)
            {
                result_.success = false;
                result_.message += " Simulation reset failed.";
            }
            complete();
            return;
        }
        if (!response->success)
        {
            finish(false, response->message);
            return;
        }
        run_id_ = response->run_id;
        if (phase_ == Phase::Preparing)
        {
            phase_ = Phase::WaitingEstimate;
            deadline_ = std::chrono::steady_clock::now() + std::chrono::seconds{5};
        }
        else if (phase_ == Phase::Starting)
        {
            start_stamp_ = response->stamp;
            phase_ = Phase::Running;
        }
    }
    if (goal_->is_canceling() && phase_ != Phase::Finishing)
    {
        finish(false, "Interception canceled.");
        return;
    }
    if (phase_ == Phase::WaitingEstimate)
    {
        bool const valid =
            estimate_ && estimate_->state.run_id == run_id_ &&
            estimate_->status == gnc_interfaces::msg::TargetEstimate::VALID &&
            estimate_->state.header.frame_id == "world" &&
            fromMsg(estimate_->state).position_m.allFinite() && interceptor_ &&
            interceptor_->run_id == run_id_ &&
            std::abs((fromMsg(interceptor_->header.stamp) - fromMsg(estimate_->state.header.stamp))
                         .count()) <= 0.25;
        if (valid)
        {
            phase_ = Phase::Starting;
            requestControl(Control::Request::START);
        }
        else if (std::chrono::steady_clock::now() >= deadline_)
            finish(false, "No valid target estimate available for launch.");
    }
    if (phase_ == Phase::Running) evaluate();
}

void InterceptionNode::stateCallback(
    gnc_interfaces::msg::InterceptorState::ConstSharedPtr interceptor,
    gnc_interfaces::msg::TargetState::ConstSharedPtr target)
{
    if (interceptor->run_id != target->run_id ||
        (interceptor_ && fromMsg(interceptor->header.stamp) <= fromMsg(interceptor_->header.stamp)))
        return;
    interceptor_ = std::move(interceptor);
    target_ = std::move(target);
    if (phase_ == Phase::Running) evaluate();
}

void InterceptionNode::evaluate()
{
    if (!interceptor_ || interceptor_->run_id < run_id_) return;
    if (interceptor_->run_id > run_id_)
    {
        finish(false, "Simulation reset during interception.");
        return;
    }
    double const distance =
        (fromMsg(*target_).position_m - fromMsg(*interceptor_).cartesian.position_m).norm();
    double const altitude = interceptor_->position_m.z;
    result_.final_distance_m = distance;
    result_.elapsed_time_s = (fromMsg(interceptor_->header.stamp) - fromMsg(start_stamp_)).count();
    bool const crashed = altitude < 0.0;
    bool const intercepted = distance < goal_->get_goal()->interception_distance_m;
    bool const timed_out = result_.elapsed_time_s >= goal_->get_goal()->max_interception_time_s;
    if (result_.elapsed_time_s >= next_feedback_s_ || crashed || intercepted ||
        goal_->is_canceling() || timed_out)
    {
        auto feedback = std::make_shared<Intercept::Feedback>();
        feedback->distance_m = distance;
        feedback->interceptor_altitude_m = altitude;
        feedback->elapsed_time_s = result_.elapsed_time_s;
        goal_->publish_feedback(feedback);
        next_feedback_s_ = result_.elapsed_time_s + 0.1;
    }
    if (goal_->is_canceling())
        finish(false, "Interception canceled.");
    else if (crashed)
        finish(false, "Interceptor crashed below ground level.");
    else if (intercepted)
        finish(true, "Target intercepted.");
    else if (timed_out)
        finish(false, "Interception time limit exceeded.");
}

void InterceptionNode::finish(bool success, std::string const& message)
{
    result_.success = success;
    result_.message = message;
    phase_ = Phase::Finishing;
    requestControl(Control::Request::RESET);
}

void InterceptionNode::complete()
{
    auto result = std::make_shared<Intercept::Result>(result_);
    if (goal_->is_canceling())
    {
        result->success = false;
        goal_->canceled(result);
    }
    else if (result->success)
        goal_->succeed(result);
    else
        goal_->abort(result);
    RCLCPP_INFO(get_logger(), "%s Distance: %.3f m; elapsed: %.2f s.", result->message.c_str(),
                result->final_distance_m, result->elapsed_time_s);
    goal_.reset();
    phase_ = Phase::Idle;
}

}  // namespace gnc_ros

RCLCPP_COMPONENTS_REGISTER_NODE(gnc_ros::InterceptionNode)
