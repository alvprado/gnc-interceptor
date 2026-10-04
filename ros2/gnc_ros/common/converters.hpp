#pragma once

#include <builtin_interfaces/msg/time.hpp>
#include <string>

#include "gnc_interfaces/msg/guidance_command.hpp"
#include "gnc_interfaces/msg/interceptor_state.hpp"
#include "gnc_interfaces/msg/radar_measurement.hpp"
#include "gnc_interfaces/msg/target_state.hpp"
#include "math/state_types.hpp"
#include "sensor_model/radar_model.hpp"
#include "simulation/uav_3dof_model.hpp"

namespace gnc_ros
{

/// @brief Convert elapsed simulation time to a ROS timestamp, rounded to nanoseconds.
/// @pre The timestamp is finite and representable by the ROS Time message.
[[nodiscard]] builtin_interfaces::msg::Time toMsg(math::Timestamp timestamp);

/// @brief Convert a ROS simulation timestamp to elapsed seconds.
[[nodiscard]] math::Timestamp fromMsg(builtin_interfaces::msg::Time const& timestamp);

/// @brief Convert vehicle state to a stamped message in the given inertial frame.
[[nodiscard]] gnc_interfaces::msg::InterceptorState toMsg(
    math::VehicleState const& state, builtin_interfaces::msg::Time const& stamp,
    std::string const& frame_id);

/// @brief Extract vehicle state; no coordinate transformation is performed.
[[nodiscard]] math::VehicleState fromMsg(gnc_interfaces::msg::InterceptorState const& message);

/// @brief Convert target state to a stamped message in the given inertial frame.
[[nodiscard]] gnc_interfaces::msg::TargetState toMsg(
    math::CartesianState const& state, builtin_interfaces::msg::Time const& stamp,
    std::string const& frame_id);

/// @brief Extract target state; no coordinate transformation is performed.
[[nodiscard]] math::CartesianState fromMsg(gnc_interfaces::msg::TargetState const& message);

/// @brief Convert a radar sample, preserving its timestamp, in the given sensor frame.
[[nodiscard]] gnc_interfaces::msg::RadarMeasurement toMsg(
    sensor_model::SensorMeasurement const& measurement, std::string const& frame_id);

/// @brief Extract a radar sample and its timestamp.
[[nodiscard]] sensor_model::SensorMeasurement fromMsg(
    gnc_interfaces::msg::RadarMeasurement const& message);

/// @brief Convert [thrust, load factor, bank angle] to a stamped guidance command.
[[nodiscard]] gnc_interfaces::msg::GuidanceCommand toMsg(
    simulation::UAV3DofModel::ControlVec const& control,
    builtin_interfaces::msg::Time const& stamp, std::string const& frame_id);

/// @brief Extract [thrust, load factor, bank angle] from a guidance command.
[[nodiscard]] simulation::UAV3DofModel::ControlVec fromMsg(
    gnc_interfaces::msg::GuidanceCommand const& message);

}  // namespace gnc_ros
