#include "simulation/parameters/sensor_config.hpp"

#include "common/parameter_utils.hpp"

namespace gnc_ros
{
namespace
{

constexpr auto prefix = "sensor";

}  // namespace

sensor_model::RadarModelConfig readSensorConfig(rclcpp::Node& node)
{
    sensor_model::RadarModelConfig config;
    config.range_var = readParameter(node, prefix, "range_var", config.range_var);
    config.range_rate_var = readParameter(node, prefix, "range_rate_var", config.range_rate_var);
    config.azimuth_var = readParameter(node, prefix, "azimuth_var", config.azimuth_var);
    config.elevation_var = readParameter(node, prefix, "elevation_var", config.elevation_var);
    return config;
}

sensor_model::RadarModel makeSensorModel(sensor_model::RadarModelConfig const& config)
{
    return sensor_model::RadarModel{config};
}

}  // namespace gnc_ros
