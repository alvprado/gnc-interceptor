#include "sensor_model/radar_model.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

#include "math/angles.hpp"

namespace sensor_model
{
namespace
{

/// @brief Floor applied to range before it's used as a division denominator,
/// avoiding a div-by-zero at (or extremely near) interception.
constexpr double min_range_m{1.0e-6};

}  // namespace

RadarModel::RadarModel(RadarModelConfig const& config, std::mt19937::result_type seed)
    : range_noise_(0.0, std::sqrt(std::max(config.range_var, 0.0)), seed),
      range_rate_noise_(0.0, std::sqrt(std::max(config.range_rate_var, 0.0)), seed + 1),
      azimuth_noise_(0.0, std::sqrt(std::max(config.azimuth_var, 0.0)), seed + 2),
      elevation_noise_(0.0, std::sqrt(std::max(config.elevation_var, 0.0)), seed + 3)
{
}

SensorMeasurement RadarModel::step(math::CartesianState const& target_state,
                                   math::VehicleState const& interceptor_state,
                                   Timestamp timestamp)
{
    Eigen::Vector3d const r_world =
        target_state.position_m - interceptor_state.cartesian_state.position_m;
    Eigen::Vector3d const v_r_world =
        target_state.velocity_mps - interceptor_state.cartesian_state.velocity_mps;

    double const real_range = r_world.norm();
    double const range_safe = std::max(real_range, min_range_m);

    double const noisy_range = real_range + range_noise_.sample();
    double const noisy_range_rate =
        (r_world.dot(v_r_world) / range_safe) + range_rate_noise_.sample();

    Eigen::Vector3d const r_body = interceptor_state.orientation.conjugate() * r_world;

    double const noisy_azimuth =
        math::wrapToPi(std::atan2(r_body.y(), r_body.x()) + azimuth_noise_.sample());
    double const noisy_elevation =
        std::atan2(r_body.z(), std::sqrt(r_body.x() * r_body.x() + r_body.y() * r_body.y())) +
        elevation_noise_.sample();

    return SensorMeasurement{timestamp, noisy_range, noisy_range_rate, noisy_azimuth,
                             noisy_elevation};
}

}  // namespace sensor_model
