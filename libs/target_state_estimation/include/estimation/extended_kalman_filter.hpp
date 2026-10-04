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

/// @brief Health status of the EKF's latest update or query.
enum class EKFStatus
{
    Valid,           ///< The estimate is valid and up to date.
    StaleTimestamp,  ///< The queried/measurement timestamp is older than the filter's current time.
    SingularInnovationCovariance,  ///< The innovation covariance couldn't be inverted; the last
                                   ///< correction was skipped.
};

/// @brief Output of a state estimate query: the estimate plus the filter's health.
struct EKFTargetStateEstimationOutput
{
    math::CartesianState target_state_estimate;  ///< The target state estimate.
    EKFStatus filter_status{EKFStatus::Valid};   ///< The filter's status as of this query.
};

/// @brief Configuration for EKFTargetStateEstimation.
struct EKFTargetStateEstimationConfig
{
    /// Process noise variance on target jerk, in (m/s^3)^2.
    double jerk_noise_var{10.0};
    /// Assumed measurement noise variance, [range, range_rate, azimuth, elevation].
    MeasurementVec sensor_noise_var{100.0, 4.0, 4.0e-6, 4.0e-6};
    /// Initial position error variance on track initiation, in m^2.
    double initial_position_var{1.0e4};
    /// Initial velocity error variance on track initiation, in (m/s)^2.
    double initial_velocity_var{4.0e4};
    /// Initial acceleration error variance on track initiation, in (m/s^2)^2.
    double initial_acceleration_var{4.0e2};
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
    /// @param[in] timestamp The time to estimate the target state at.
    /// @returns The estimated target state at timestamp, plus the filter's status.
    [[nodiscard]] EKFTargetStateEstimationOutput stateEstimateAt(math::Timestamp timestamp) const;

    /// @brief Advance the filter with a new measurement: predicts the state
    /// to the measurement's timestamp, then corrects it with the measurement.
    /// @param[in] measurement The noisy radar measurement of the target.
    /// @param[in] interceptor_state The interceptor's ground-truth vehicle
    /// state at the time of the measurement
    void processMeasurement(sensor_model::SensorMeasurement const& measurement,
                            math::VehicleState const& interceptor_state);

    /// @brief Query the error covariance at a given time.
    /// @param[in] timestamp The time to estimate the error covariance at.
    /// @returns The predicted error covariance at timestamp.
    [[nodiscard]] StateCov errorCovarianceAt(math::Timestamp timestamp) const;

    /// @brief Whether the track has been initialized by a first measurement.
    [[nodiscard]] bool isInitialized() const noexcept;

private:
    /// @brief Initialize the track from the first measurement ever received.
    /// @param[in] measurement The first radar measurement of the target.
    /// @param[in] interceptor_state The interceptor's ground-truth vehicle
    /// state at the time of the measurement.
    void initializeTrack(sensor_model::SensorMeasurement const& measurement,
                         math::VehicleState const& interceptor_state);

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

    /// @brief The nonlinear measurement function h(x): predicts the
    /// [range, range_rate, azimuth, elevation] measurement a given state and
    /// interceptor pose would produce, noise-free.
    /// @param[in] target_state_estimate The state to evaluate the measurement function at.
    /// @param[in] interceptor_state The interceptor's ground-truth vehicle
    /// state the measurement is taken relative to.
    /// @returns The predicted noise-free measurement.
    [[nodiscard]] MeasurementVec measurementModel(
        StateVec const& target_state_estimate, math::VehicleState const& interceptor_state) const;

    /// @brief Split the internal [pos, vel, accel] state vector into a
    /// CartesianState.
    /// @param[in] state_estimate The state to convert.
    /// @returns The equivalent CartesianState.
    [[nodiscard]] math::CartesianState convertInternalStateToCartesian(
        StateVec const& state_estimate) const;

    double jerk_noise_var_;                 ///< Process noise variance on target jerk.
    MeasurementCov measurement_noise_cov_;  ///< Assumed measurement noise covariance (diagonal).
    double initial_position_var_;      ///< Initial position error variance on track initiation.
    double initial_velocity_var_;      ///< Initial velocity error variance on track initiation.
    double initial_acceleration_var_;  ///< Initial acceleration error variance on track initiation.

    bool initialized_{false};                    ///< Whether the track has been initialized.
    EKFStatus filter_status_{EKFStatus::Valid};  ///< Status of the filter's latest predict/correct.
    math::Timestamp filter_time_{0.0};           ///< Time of the current state estimate.
    StateVec state_estimate_{StateVec::Zero()};  ///< The current target state estimate.
    StateCov error_cov_{StateCov::Zero()};       ///< The current error covariance matrix
};
}  // namespace estimation
