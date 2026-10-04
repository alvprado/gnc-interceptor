#include "simulation/simulation_node.hpp"

#include <chrono>
#include <rclcpp_components/register_node_macro.hpp>
#include <utility>

#include "common/converters.hpp"

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
    reset_service_ = create_service<gnc_interfaces::srv::ResetSimulation>(
        "simulation/reset",
        [](gnc_interfaces::srv::ResetSimulation::Request::SharedPtr,
           gnc_interfaces::srv::ResetSimulation::Response::SharedPtr response)
        {
            response->success = false;
            response->message = "Reset is not implemented in the simulation skeleton.";
        });

    simulation_timer_ = create_wall_timer(std::chrono::duration<double>{sim_config_.dt_s},
                                          [this] { simulationCallback(); });
    RCLCPP_INFO(get_logger(), "Simulation timer ready with dt = %.6f s.",
                sim_config_.dt_s);
}

void SimulationNode::guidanceCommandCallback(
    gnc_interfaces::msg::GuidanceCommand::ConstSharedPtr command)
{
    command_ = std::move(command);
}

void SimulationNode::simulationCallback()
{
    math::Timestamp const simulation_time{static_cast<double>(step_count_) * sim_config_.dt_s};
    auto const stamp = toMsg(simulation_time);
    auto const target_gt_state = target_trajectory_.evaluateTargetStateAt(simulation_time.count());
    auto const measurement = sensor_.step(target_gt_state, interceptor_state_, simulation_time);

    rosgraph_msgs::msg::Clock clock;
    clock.clock = stamp;
    clock_publisher_->publish(clock);
    target_publisher_->publish(toMsg(target_gt_state, stamp, "world"));
    interceptor_publisher_->publish(toMsg(interceptor_state_, stamp, "world"));
    radar_publisher_->publish(toMsg(measurement, "interceptor_body"));

    // Hold the initial state until guidance supplies a command.
    if (command_)
    {
        interceptor_state_ =
            interceptor_.step(interceptor_state_, fromMsg(*command_), sim_config_.dt_s);
    }
    ++step_count_;
}

}  // namespace gnc_ros

RCLCPP_COMPONENTS_REGISTER_NODE(gnc_ros::SimulationNode)
