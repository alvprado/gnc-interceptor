#pragma once

#include "sensor_model/radar_model.hpp"

namespace rclcpp
{
class Node;
}

namespace gnc_ros
{

/// @brief Declare and read the radar's startup-only ROS parameters.
[[nodiscard]] sensor_model::RadarModelConfig readSensorConfig(rclcpp::Node& node);

/// @brief Construct the radar model from its configuration.
[[nodiscard]] sensor_model::RadarModel makeSensorModel(sensor_model::RadarModelConfig const& config);

}  // namespace gnc_ros
