#include "common/converters.hpp"

#include <chrono>
#include <cstdint>

namespace gnc_ros
{
namespace
{

template <typename Message>
void writeCartesianState(math::CartesianState const& state, Message& message)
{
    message.position_m.x = state.position_m.x();
    message.position_m.y = state.position_m.y();
    message.position_m.z = state.position_m.z();
    message.velocity_mps.x = state.velocity_mps.x();
    message.velocity_mps.y = state.velocity_mps.y();
    message.velocity_mps.z = state.velocity_mps.z();
    message.acceleration_mps2.x = state.acceleration_mps2.x();
    message.acceleration_mps2.y = state.acceleration_mps2.y();
    message.acceleration_mps2.z = state.acceleration_mps2.z();
}

template <typename Message>
[[nodiscard]] math::CartesianState readCartesianState(Message const& message)
{
    return {{message.position_m.x, message.position_m.y, message.position_m.z},
            {message.velocity_mps.x, message.velocity_mps.y, message.velocity_mps.z},
            {message.acceleration_mps2.x, message.acceleration_mps2.y,
             message.acceleration_mps2.z}};
}

}  // namespace

builtin_interfaces::msg::Time toMsg(math::Timestamp timestamp)
{
    auto const nanoseconds = std::chrono::round<std::chrono::nanoseconds>(timestamp);
    auto const seconds = std::chrono::floor<std::chrono::seconds>(nanoseconds);
    builtin_interfaces::msg::Time message;
    message.sec = static_cast<std::int32_t>(seconds.count());
    message.nanosec = static_cast<std::uint32_t>((nanoseconds - seconds).count());
    return message;
}

math::Timestamp fromMsg(builtin_interfaces::msg::Time const& timestamp)
{
    return std::chrono::seconds{timestamp.sec} + std::chrono::nanoseconds{timestamp.nanosec};
}

gnc_interfaces::msg::InterceptorState toMsg(
    math::VehicleState const& state, builtin_interfaces::msg::Time const& stamp,
    std::string const& frame_id)
{
    gnc_interfaces::msg::InterceptorState message;
    message.header.stamp = stamp;
    message.header.frame_id = frame_id;
    writeCartesianState(state.cartesian, message);
    message.attitude.x = state.attitude.x();
    message.attitude.y = state.attitude.y();
    message.attitude.z = state.attitude.z();
    message.attitude.w = state.attitude.w();
    return message;
}

math::VehicleState fromMsg(gnc_interfaces::msg::InterceptorState const& message)
{
    return {readCartesianState(message),
            Eigen::Quaterniond{message.attitude.w, message.attitude.x,
                               message.attitude.y, message.attitude.z}};
}

gnc_interfaces::msg::TargetState toMsg(
    math::CartesianState const& state, builtin_interfaces::msg::Time const& stamp,
    std::string const& frame_id)
{
    gnc_interfaces::msg::TargetState message;
    message.header.stamp = stamp;
    message.header.frame_id = frame_id;
    writeCartesianState(state, message);
    return message;
}

math::CartesianState fromMsg(gnc_interfaces::msg::TargetState const& message)
{
    return readCartesianState(message);
}

gnc_interfaces::msg::RadarMeasurement toMsg(
    sensor_model::SensorMeasurement const& measurement, std::string const& frame_id)
{
    gnc_interfaces::msg::RadarMeasurement message;
    message.header.stamp = toMsg(measurement.timestamp);
    message.header.frame_id = frame_id;
    message.range_m = measurement.range_m;
    message.range_rate_mps = measurement.range_rate_mps;
    message.azimuth_rad = measurement.azimuth_rad;
    message.elevation_rad = measurement.elevation_rad;
    return message;
}

sensor_model::SensorMeasurement fromMsg(gnc_interfaces::msg::RadarMeasurement const& message)
{
    return {fromMsg(message.header.stamp), message.range_m, message.range_rate_mps,
            message.azimuth_rad, message.elevation_rad};
}

gnc_interfaces::msg::GuidanceCommand toMsg(
    simulation::UAV3DofModel::ControlVec const& control,
    builtin_interfaces::msg::Time const& stamp, std::string const& frame_id)
{
    gnc_interfaces::msg::GuidanceCommand message;
    message.header.stamp = stamp;
    message.header.frame_id = frame_id;
    message.thrust_n = control[0];
    message.load_factor = control[1];
    message.bank_angle_rad = control[2];
    return message;
}

simulation::UAV3DofModel::ControlVec fromMsg(gnc_interfaces::msg::GuidanceCommand const& message)
{
    return {message.thrust_n, message.load_factor, message.bank_angle_rad};
}

}  // namespace gnc_ros
