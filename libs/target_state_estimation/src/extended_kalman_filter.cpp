#include "estimation/extended_kalman_filter.hpp"

#include <Eigen/src/Core/ArithmeticSequence.h>
#include <Eigen/src/Core/Matrix.h>

#include <cmath>

#include "math/angles.hpp"
#include "math/state_types.hpp"

namespace estimation
{

EKFTargetStateEstimation::EKFTargetStateEstimation(EKFTargetStateEstimationConfig const& config)
    : jerk_noise_var_(config.jerk_noise_var),
      measurement_noise_cov_(config.sensor_noise_var.asDiagonal()),
      initial_position_var_(config.initial_position_var),
      initial_velocity_var_(config.initial_velocity_var),
      initial_acceleration_var_(config.initial_acceleration_var)
{
}

EKFTargetStateEstimationOutput EKFTargetStateEstimation::stateEstimateAt(
    math::Timestamp timestamp) const
{
    double const dt = timestamp.count() - filter_time_.count();
    if (dt < 0.0)
    {
        return {convertInternalStateToCartesian(state_estimate_), EKFStatus::StaleTimestamp};
    }
    StateJacobian const F = stateTransitionMatrix(dt);
    return {convertInternalStateToCartesian(F * state_estimate_), filter_status_};
}

StateCov EKFTargetStateEstimation::errorCovarianceAt(math::Timestamp timestamp) const
{
    double const dt = timestamp.count() - filter_time_.count();
    if (dt < 0.0)
    {
        return error_cov_;
    }
    StateJacobian const F = stateTransitionMatrix(dt);
    StateCov const Q = processNoiseCovariance(dt);
    StateCov error_cov_pred = F * error_cov_ * F.transpose() + Q;
    return error_cov_pred;
}

bool EKFTargetStateEstimation::isInitialized() const noexcept
{
    return initialized_;
}

void EKFTargetStateEstimation::processMeasurement(
    sensor_model::SensorMeasurement const& measurement, math::VehicleState const& interceptor_state)
{
    if (!initialized_)
    {
        initializeTrack(measurement, interceptor_state);
        return;
    }

    double const dt = measurement.timestamp.count() - filter_time_.count();
    if (dt < 0)
    {
        filter_status_ = EKFStatus::StaleTimestamp;
        return;
    }

    predictionStep(measurement.timestamp);
    correctionStep(measurement, interceptor_state);
}

void EKFTargetStateEstimation::initializeTrack(sensor_model::SensorMeasurement const& measurement,
                                               math::VehicleState const& interceptor_state)
{
    double const cos_elevation = std::cos(measurement.elevation_rad);
    Eigen::Vector3d const r_B{
        measurement.range_m * cos_elevation * std::cos(measurement.azimuth_rad),
        measurement.range_m * cos_elevation * std::sin(measurement.azimuth_rad),
        measurement.range_m * std::sin(measurement.elevation_rad)};

    state_estimate_ = StateVec::Zero();
    state_estimate_.head<3>() =
        interceptor_state.attitude * r_B + interceptor_state.cartesian.position_m;

    error_cov_ = StateCov::Zero();
    error_cov_(Eigen::seqN(0, 3), Eigen::seqN(0, 3)) =
        initial_position_var_ * Eigen::Matrix3d::Identity();
    error_cov_(Eigen::seqN(3, 3), Eigen::seqN(3, 3)) =
        initial_velocity_var_ * Eigen::Matrix3d::Identity();
    error_cov_(Eigen::seqN(6, 3), Eigen::seqN(6, 3)) =
        initial_acceleration_var_ * Eigen::Matrix3d::Identity();

    filter_time_ = measurement.timestamp;
    initialized_ = true;
}

void EKFTargetStateEstimation::predictionStep(math::Timestamp timestamp)
{
    double const dt = timestamp.count() - filter_time_.count();
    if (dt < 0.0)
    {
        filter_status_ = EKFStatus::StaleTimestamp;
        return;
    }

    StateJacobian const F = stateTransitionMatrix(dt);
    state_estimate_ = F * state_estimate_;

    StateCov const Q = processNoiseCovariance(dt);
    error_cov_ = F * error_cov_ * F.transpose() + Q;

    filter_time_ = timestamp;
    filter_status_ = EKFStatus::Valid;
}

void EKFTargetStateEstimation::correctionStep(sensor_model::SensorMeasurement const& measurement,
                                              math::VehicleState const& interceptor_state)
{
    MeasurementJacobian const H = measurementJacobian(state_estimate_, interceptor_state);

    MeasurementCov const S = H * error_cov_ * H.transpose() + measurement_noise_cov_;
    Eigen::LDLT<MeasurementCov> ldlt(S);
    if (ldlt.info() != Eigen::Success)
    {
        filter_status_ = EKFStatus::SingularInnovationCovariance;
        return;
    }
    MeasurementCov const S_inv = ldlt.solve(MeasurementCov::Identity());

    Eigen::Matrix<double, 9, 4> const kalman_gain = error_cov_ * H.transpose() * S_inv;

    MeasurementVec const z{measurement.range_m, measurement.range_rate_mps, measurement.azimuth_rad,
                           measurement.elevation_rad};

    MeasurementVec y = z - measurementModel(state_estimate_, interceptor_state);
    y[2] = math::wrapToPi(y[2]);

    state_estimate_ += kalman_gain * y;
    error_cov_ -= kalman_gain * H * error_cov_;

    filter_status_ = EKFStatus::Valid;
}

StateJacobian EKFTargetStateEstimation::stateTransitionMatrix(double dt) const
{
    StateJacobian state_jacobian{StateJacobian::Identity()};
    Eigen::Matrix3d const identity3{Eigen::Matrix3d::Identity()};
    state_jacobian(Eigen::seqN(0, 3), Eigen::seqN(3, 3)) = dt * identity3;
    state_jacobian(Eigen::seqN(0, 3), Eigen::seqN(6, 3)) = 0.5 * dt * dt * identity3;
    state_jacobian(Eigen::seqN(3, 3), Eigen::seqN(6, 3)) = dt * identity3;

    return state_jacobian;
}

StateCov EKFTargetStateEstimation::processNoiseCovariance(double dt) const
{
    double const dt2 = dt * dt;
    double const dt3 = dt2 * dt;
    double const dt4 = dt3 * dt;
    double const dt5 = dt4 * dt;
    Eigen::Matrix3d const identity3{Eigen::Matrix3d::Identity()};

    StateCov process_noise_cov{StateCov::Zero()};
    process_noise_cov(Eigen::seqN(0, 3), Eigen::seqN(0, 3)) = (dt5 / 20.0) * identity3;
    process_noise_cov(Eigen::seqN(0, 3), Eigen::seqN(3, 3)) = (dt4 / 8.0) * identity3;
    process_noise_cov(Eigen::seqN(0, 3), Eigen::seqN(6, 3)) = (dt3 / 6.0) * identity3;
    process_noise_cov(Eigen::seqN(3, 3), Eigen::seqN(3, 3)) = (dt3 / 3.0) * identity3;
    process_noise_cov(Eigen::seqN(3, 3), Eigen::seqN(6, 3)) = (dt2 / 2.0) * identity3;
    process_noise_cov(Eigen::seqN(6, 3), Eigen::seqN(6, 3)) = dt * identity3;

    process_noise_cov(Eigen::seqN(3, 3), Eigen::seqN(0, 3)) =
        process_noise_cov(Eigen::seqN(0, 3), Eigen::seqN(3, 3)).transpose();
    process_noise_cov(Eigen::seqN(6, 3), Eigen::seqN(0, 3)) =
        process_noise_cov(Eigen::seqN(0, 3), Eigen::seqN(6, 3)).transpose();
    process_noise_cov(Eigen::seqN(6, 3), Eigen::seqN(3, 3)) =
        process_noise_cov(Eigen::seqN(3, 3), Eigen::seqN(6, 3)).transpose();

    return jerk_noise_var_ * process_noise_cov;
}

MeasurementJacobian EKFTargetStateEstimation::measurementJacobian(
    StateVec const& state, math::VehicleState const& interceptor_state) const
{
    /// Initialize measurement jacobian grad_h/grad_x to zeros
    MeasurementJacobian measurement_jacobian{MeasurementJacobian::Zero()};

    /// Relative position vector
    Eigen::Vector3d const r = state.head<3>() - interceptor_state.cartesian.position_m;

    /// Range, safeguard for protection agains zero division
    double const range = r.norm();
    constexpr double min_distance{10.0e-6};
    double const safe_range = std::max(range, min_distance);

    /// Relative position direction/unit vector
    Eigen::Vector3d const r_unit = r / safe_range;

    /// Relative velocity
    Eigen::Vector3d const v_r = state(Eigen::seqN(3, 3)) - interceptor_state.cartesian.velocity_mps;

    /// The gradient of the range rate w.r.t. the target position
    Eigen::Vector3d const grad_h_range_rate_wrt_position =
        (Eigen::Matrix3d::Identity() - r_unit * r_unit.transpose()) * v_r / safe_range;

    /// Relative position in body frame. Attitude is equivalent to the rotation from body to
    /// world frame R_WB, such that from world to body rotation requires its conjugate quaternion
    Eigen::Vector3d const r_B = interceptor_state.attitude.conjugate() * r;

    /// Helper variables, the square norm of the relative heading vector on the planar plane from
    /// the body frame
    double const rel_heading = r_B.x() * r_B.x() + r_B.y() * r_B.y();
    double const rel_heading_safe = std::max(rel_heading, min_distance);

    /// The gradient of the azimuth w.r.t. the target position in the world frame, notice rotation
    /// back to world frame
    Eigen::Vector3d const grad_h_azimuth_wrt_position =
        interceptor_state.attitude *
        Eigen::Vector3d{-r_B.y() / rel_heading_safe, r_B.x() / rel_heading_safe, 0.0};

    /// The gradient of the elevation w.r.t. the target position in the world frame, notice rotation
    /// back to world frame
    Eigen::Vector3d const grad_h_elevation_wrt_position =
        interceptor_state.attitude *
        Eigen::Vector3d{
            -(r_B.x() * r_B.z()) / (safe_range * safe_range * std::sqrt(rel_heading_safe)),
            -(r_B.y() * r_B.z()) / (safe_range * safe_range * std::sqrt(rel_heading_safe)),
            std::sqrt(rel_heading_safe) / (safe_range * safe_range)};

    /// Fill measurement jacobian matrix
    measurement_jacobian(Eigen::seqN(0, 1), Eigen::seqN(0, 3)) = r_unit.transpose();
    measurement_jacobian(Eigen::seqN(1, 1), Eigen::seqN(0, 3)) =
        grad_h_range_rate_wrt_position.transpose();
    measurement_jacobian(Eigen::seqN(1, 1), Eigen::seqN(3, 3)) = r_unit.transpose();
    measurement_jacobian(Eigen::seqN(2, 1), Eigen::seqN(0, 3)) =
        grad_h_azimuth_wrt_position.transpose();
    measurement_jacobian(Eigen::seqN(3, 1), Eigen::seqN(0, 3)) =
        grad_h_elevation_wrt_position.transpose();

    return measurement_jacobian;
}

MeasurementVec EKFTargetStateEstimation::measurementModel(
    StateVec const& target_state_estimate, math::VehicleState const& interceptor_state) const
{
    Eigen::Vector3d const r =
        target_state_estimate.head<3>() - interceptor_state.cartesian.position_m;
    Eigen::Vector3d const v_r =
        target_state_estimate(Eigen::seqN(3, 3)) - interceptor_state.cartesian.velocity_mps;

    double const range = r.norm();
    constexpr double min_distance{10.0e-6};
    double const safe_range = std::max(range, min_distance);

    double const range_rate = r.dot(v_r) / safe_range;

    Eigen::Vector3d const r_B = interceptor_state.attitude.conjugate() * r;
    double const azimuth = std::atan2(r_B.y(), r_B.x());
    double const elevation = std::atan2(r_B.z(), std::sqrt(r_B.x() * r_B.x() + r_B.y() * r_B.y()));

    return MeasurementVec{range, range_rate, azimuth, elevation};
}

math::CartesianState EKFTargetStateEstimation::convertInternalStateToCartesian(
    StateVec const& state_estimate) const
{
    return math::CartesianState{.position_m = state_estimate.head<3>(),
                                .velocity_mps = state_estimate(Eigen::seqN(3, 3)),
                                .acceleration_mps2 = state_estimate(Eigen::seqN(6, 3))};
}

}  // namespace estimation
