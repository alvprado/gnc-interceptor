#pragma once

#include <Eigen/Dense>

#include "math/state_types.hpp"
#include "sensor_model/radar_model.hpp"

namespace estimation
{

/// @brief Target state: [x, y, z, vx, vy, vz, ax, ay, az]^T, position,
/// velocity and acceleration in the inertial frame.
using StateVec = Eigen::Vector<double, 9>;
/// @brief Covariance of StateVec.
using StateCov = Eigen::Matrix<double, 9, 9>;
/// @brief Jacobian of the discrete state transition with respect to StateVec.
using StateJacobian = StateCov;
/// @brief Measurement vector: [range, range_rate, azimuth, elevation]^T,
/// matching sensor_model::SensorMeasurement.
using MeasurementVec = Eigen::Vector<double, 4>;
/// @brief Covariance of MeasurementVec.
using MeasurementCov = Eigen::Matrix<double, 4, 4>;
/// @brief Jacobian of the measurement function with respect to StateVec.
using MeasurementJacobian = Eigen::Matrix<double, 4, 9>;

/// @brief Configuration for EKFTargetStateEstimation.
struct EKFTargetStateEstimationConfig
{
    double jerk_noise_var{1.0};  ///< Process noise variance on target jerk, in (m/s^3)^2.
    /// Assumed measurement noise variance, [range, range_rate, azimuth, elevation].
    MeasurementVec sensor_noise_var{1.0, 1.0, 1.0, 1.0};
};

/// @brief Extended Kalman filter estimating the target's Cartesian state from
/// noisy radar measurements, under a constant-acceleration motion model with
/// white-noise jerk as process noise.
class EKFTargetStateEstimation
{
public:
    /// @brief Construct a filter.
    /// @param[in] config The process and measurement noise configuration.
    explicit EKFTargetStateEstimation(EKFTargetStateEstimationConfig const& config);

    /// @brief Query the target state estimate at a given time.
    /// @details A pure read: propagates a copy of the current estimate to
    /// timestamp without modifying the filter's own state.
    /// @param[in] timestamp The time to estimate the target state at.
    /// @returns The estimated target position and velocity at timestamp.
    [[nodiscard]] math::CartesianState estimateAt(math::Timestamp timestamp) const;

    /// @brief Advance the filter with a new measurement: predicts the state
    /// to the measurement's timestamp, then corrects it with the measurement.
    /// @param[in] measurement The noisy radar measurement of the target.
    /// @param[in] interceptor_state The interceptor's ground-truth vehicle
    /// state at the time of the measurement
    void processMeasurement(sensor_model::SensorMeasurement const& measurement,
                            math::VehicleState const& interceptor_state);

private:
    /// @brief Propagate the state estimate and covariance to timestamp.
    /// @param[in] timestamp The time to predict the state to.
    void predictionStep(math::Timestamp timestamp);

    /// @brief Correct the state estimate and covariance with a measurement.
    /// @param[in] measurement The noisy radar measurement of the target.
    /// @param[in] interceptor_state The interceptor's ground-truth vehicle
    /// state at the time of the measurement.
    void correctionStep(sensor_model::SensorMeasurement const& measurement,
                        math::VehicleState const& interceptor_state);

    /// @brief The discrete state transition Jacobian for a timestep dt.
    /// @param[in] dt The timestep, in seconds.
    /// @returns The Jacobian of the state transition w.r.t. the state.
    [[nodiscard]] StateJacobian stateTransitionMatrix(double dt) const;

    /// @brief The discrete process noise covariance for a timestep dt.
    /// @details Continuous white-noise-jerk model: jerk is white noise with
    /// a power spectral density of jerk_noise_var, propagated through the state
    // transition dynamics.
    /// @param[in] dt The timestep in seconds.
    /// @returns The process noise covariance, driven by jerk_noise_var_.
    [[nodiscard]] StateCov processNoiseCovariance(double dt) const;

    /// @brief The measurement function's Jacobian at a given state and
    /// interceptor pose.
    /// @param[in] state The state to linearize the measurement function at.
    /// @param[in] interceptor_state The interceptor's ground-truth vehicle
    /// state the measurement is taken relative to.
    /// @returns The Jacobian of the measurement function w.r.t. the state.
    [[nodiscard]] MeasurementJacobian measurementJacobian(
        StateVec const& state, math::VehicleState const& interceptor_state) const;

    double jerk_noise_var_;                 ///< Process noise variance on target jerk.
    MeasurementCov measurement_noise_cov_;  ///< Assumed measurement noise covariance (diagonal).

    math::Timestamp filter_time_{0.0};  ///< Time of the current state estimate.
    StateVec state_estimate_{};         ///< The current target state estimate.
};
}  // namespace estimation
