#pragma once

#include <chrono>
#include <random>

#include "math/state_types.hpp"
#include "sensor_model/noise_generator.hpp"

namespace sensor_model
{

/// @brief Per-channel measurement noise variance for RadarModel.
struct RadarModelConfig
{
    double range_var{1.0};       ///< Range measurement noise variance, in m^2.
    double range_rate_var{1.0};  ///< Range-rate measurement noise variance, in (m/s)^2.
    double azimuth_var{1.0};     ///< Azimuth measurement noise variance, in rad^2.
    double elevation_var{1.0};   ///< Elevation measurement noise variance, in rad^2.
};

/// @brief Simulation time, in seconds since the run started.
using Timestamp = std::chrono::duration<double>;

/// @brief A single radar measurement: range, range rate, azimuth and elevation.
struct SensorMeasurement
{
    Timestamp timestamp{0.0};    ///< Simulation time this measurement was taken at.
    double range_m{0.0};         ///< Slant range to the target, in m.
    double range_rate_mps{0.0};  ///< Closing (negative) / opening (positive) rate, in m/s.
    double azimuth_rad{0.0};     ///< Bearing to the target in the sensor's body frame, in rad.
    double elevation_rad{0.0};   ///< Elevation of the target in the sensor's body frame, in rad.
};

/// @brief Radar sensor model: converts target/interceptor ground-truth states
/// into a noisy [range, range rate, azimuth, elevation] measurement, with
/// azimuth/elevation resolved in the interceptor's body frame.
/// @details Not copyable (holds one move-only noise generator per channel, so
/// their sequences stay independent of each other); movable.
class RadarModel
{
public:
    /// @brief Construct a radar model.
    /// @param[in] config Per-channel measurement noise variances.
    /// @param[in] seed Base seed for the four noise channels (each channel is
    /// seeded from a distinct offset of it, so they stay independent);
    /// defaults to a nondeterministic seed, but tests can pass a fixed value
    /// for reproducible measurements.
    explicit RadarModel(RadarModelConfig const& config,
                        std::mt19937::result_type seed = std::random_device{}());

    // Move-only: avoid copying state of random engine
    RadarModel(RadarModel const&) = delete;
    RadarModel& operator=(RadarModel const&) = delete;
    RadarModel(RadarModel&&) = default;
    RadarModel& operator=(RadarModel&&) = default;

    /// @brief Compute a noisy measurement of the target, from the interceptor.
    /// @param[in] target_state The target's ground-truth Cartesian state.
    /// @param[in] interceptor_state The interceptor's ground-truth vehicle
    /// state (position, velocity and body orientation).
    /// @param[in] timestamp The simulation time this measurement is taken at,
    /// carried into the returned SensorMeasurement unchanged.
    /// @returns The noisy [range, range rate, azimuth, elevation] measurement.
    [[nodiscard]] SensorMeasurement step(math::CartesianState const& target_state,
                                         math::VehicleState const& interceptor_state,
                                         Timestamp timestamp);

private:
    GaussianNoiseGenerator range_noise_;
    GaussianNoiseGenerator range_rate_noise_;
    GaussianNoiseGenerator azimuth_noise_;
    GaussianNoiseGenerator elevation_noise_;
};

}  // namespace sensor_model
