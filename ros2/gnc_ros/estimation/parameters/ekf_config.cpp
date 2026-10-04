#include "estimation/parameters/ekf_config.hpp"

#include <vector>

#include "common/parameter_utils.hpp"

namespace gnc_ros
{
namespace
{

constexpr auto prefix = "ekf";

}  // namespace

estimation::EKFTargetStateEstimationConfig readEkfConfig(rclcpp::Node& node)
{
    estimation::EKFTargetStateEstimationConfig config;
    config.jerk_noise_var = readParameter(node, prefix, "jerk_noise_var", config.jerk_noise_var);

    auto const sensor_noise_var = readParameter(
        node, prefix, "sensor_noise_var",
        std::vector<double>{config.sensor_noise_var[0], config.sensor_noise_var[1],
                            config.sensor_noise_var[2], config.sensor_noise_var[3]});
    config.sensor_noise_var = estimation::MeasurementVec{
        sensor_noise_var[0], sensor_noise_var[1], sensor_noise_var[2], sensor_noise_var[3]};

    config.initial_position_var =
        readParameter(node, prefix, "initial_position_var", config.initial_position_var);
    config.initial_velocity_var =
        readParameter(node, prefix, "initial_velocity_var", config.initial_velocity_var);
    config.initial_acceleration_var =
        readParameter(node, prefix, "initial_acceleration_var", config.initial_acceleration_var);
    return config;
}

estimation::EKFTargetStateEstimation makeEstimator(
    estimation::EKFTargetStateEstimationConfig const& config)
{
    return estimation::EKFTargetStateEstimation{config};
}

}  // namespace gnc_ros
