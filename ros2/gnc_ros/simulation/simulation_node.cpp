#include "simulation/simulation_node.hpp"

#include <chrono>
#include <cmath>
#include <rclcpp_components/register_node_macro.hpp>
#include <utility>

#include "common/converters.hpp"
#include "math/angles.hpp"

namespace gnc_ros
{

SimulationNode::SimulationNode(rclcpp::NodeOptions const& options)
    : Node("simulation_node", options),
      sim_config_(readSimulationConfig(*this)),
      interceptor_(makeInterceptorModel(readInterceptorConfig(*this)), sim_config_.integrator),
      sensor_(makeSensorModel(readSensorConfig(*this))),
      target_trajectory_(makeTargetTrajectory(readTargetConfig(*this)))
{
    auto const qos = rclcpp::QoS{10};
    interceptor_publisher_ =
        create_publisher<gnc_interfaces::msg::InterceptorState>("interceptor/state", qos);
    target_publisher_ =
        create_publisher<gnc_interfaces::msg::TargetState>("target/ground_truth", qos);
    radar_publisher_ =
        create_publisher<gnc_interfaces::msg::RadarMeasurement>("radar/measurement", qos);
    clock_publisher_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS{});
    command_subscription_ = create_subscription<gnc_interfaces::msg::GuidanceCommand>(
        "guidance/command", qos,
        [this](gnc_interfaces::msg::GuidanceCommand::ConstSharedPtr command)
        { guidanceCommandCallback(std::move(command)); });
    control_service_ = create_service<gnc_interfaces::srv::SimulationControl>(
        "simulation/control",
        [this](gnc_interfaces::srv::SimulationControl::Request::SharedPtr request,
               gnc_interfaces::srv::SimulationControl::Response::SharedPtr response)
        { controlCallback(std::move(request), std::move(response)); });

    simulation_timer_ = create_wall_timer(std::chrono::duration<double>{sim_config_.dt_s},
                                          [this] { simulationCallback(); });
    RCLCPP_INFO(get_logger(),
                "Simulation ready with dt = %.6f s; waiting for an interception goal.",
                sim_config_.dt_s);
}

void SimulationNode::guidanceCommandCallback(
    gnc_interfaces::msg::GuidanceCommand::ConstSharedPtr command)
{
    if (running_ && command->run_id == run_id_ && fromMsg(*command).allFinite())
    {
        command_ = std::move(command);
    }
}

bool SimulationNode::start(Eigen::Vector3d const& target_position_m, std::uint64_t expected_run_id)
{
    Eigen::Vector3d const direction = target_position_m - interceptor_state_.cartesian.position_m;
    if (running_ || expected_run_id != run_id_ || !direction.allFinite() ||
        direction.norm() < 1.0e-6)
        return false;

    double const heading = std::atan2(direction.y(), direction.x());
    double const climb = std::atan2(direction.z(), std::hypot(direction.x(), direction.y()));
    interceptor_state_.attitude = math::attitudeFromHeadingPitchBank(heading, climb, 0.0);
    interceptor_state_.cartesian.velocity_mps.setZero();
    interceptor_state_.cartesian.acceleration_mps2.setZero();
    command_.reset();
    ++run_id_;
    running_ = true;
    return true;
}

void SimulationNode::pause()
{
    running_ = false;
    command_.reset();
}

void SimulationNode::reset()
{
    pause();
    ++run_id_;
    run_step_count_ = 0;
    interceptor_state_ = math::VehicleState{};
}

void SimulationNode::controlCallback(
    gnc_interfaces::srv::SimulationControl::Request::SharedPtr request,
    gnc_interfaces::srv::SimulationControl::Response::SharedPtr response)
{
    using Request = gnc_interfaces::srv::SimulationControl::Request;
    response->success = true;
    switch (request->command)
    {
        case Request::START:
            response->success =
                start(Eigen::Vector3d{request->target_position_m.x, request->target_position_m.y,
                                      request->target_position_m.z},
                      request->expected_run_id);
            break;
        case Request::PAUSE:
            pause();
            break;
        case Request::RESET:
            reset();
            break;
        default:
            response->success = false;
            break;
    }
    response->message = response->success ? "Simulation control applied."
                                          : "Invalid simulation control or start state.";
    response->run_id = run_id_;
    response->stamp = toMsg(math::Timestamp{static_cast<double>(step_count_) * sim_config_.dt_s});
}

void SimulationNode::publishScene(math::CartesianState const& target, math::Timestamp timestamp)
{
    auto const stamp = toMsg(timestamp);
    auto const measurement = sensor_.step(target, interceptor_state_, timestamp);

    rosgraph_msgs::msg::Clock clock;
    clock.clock = stamp;
    clock_publisher_->publish(clock);
    auto target_message = toMsg(target, stamp, "world");
    target_message.run_id = run_id_;
    target_publisher_->publish(target_message);
    auto interceptor_message = toMsg(interceptor_state_, stamp, "world");
    interceptor_message.run_id = run_id_;
    interceptor_publisher_->publish(interceptor_message);
    auto radar_message = toMsg(measurement, "interceptor_body");
    radar_message.run_id = run_id_;
    radar_publisher_->publish(radar_message);
}

void SimulationNode::simulationCallback()
{
    auto const target = target_trajectory_.evaluateTargetStateAt(
        static_cast<double>(run_step_count_) * sim_config_.dt_s);
    publishScene(target, math::Timestamp{static_cast<double>(step_count_) * sim_config_.dt_s});
    if (running_)
    {
        if (command_)
        {
            interceptor_state_ =
                interceptor_.step(interceptor_state_, fromMsg(*command_), sim_config_.dt_s);
        }
        ++run_step_count_;
    }
    ++step_count_;
}

}  // namespace gnc_ros

RCLCPP_COMPONENTS_REGISTER_NODE(gnc_ros::SimulationNode)
