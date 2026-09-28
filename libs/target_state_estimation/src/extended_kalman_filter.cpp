#include "estimation/extended_kalman_filter.hpp"

namespace estimation
{

EKFTargetStateEstimation::EKFTargetStateEstimation(EKFTargetStateEstimationConfig const& config)
    : jerk_noise_var_(config.jerk_noise_var),
      measurement_noise_cov_(config.sensor_noise_var.asDiagonal())
{
}

math::CartesianState EKFTargetStateEstimation::estimateAt(math::Timestamp timestamp) const
{
    /// TO-DO: implement
    return math::CartesianState{};
}

void EKFTargetStateEstimation::processMeasurement(
    sensor_model::SensorMeasurement const& measurement, math::VehicleState const& interceptor_state)
{
}

void EKFTargetStateEstimation::predictionStep(math::Timestamp timestamp) {}

void EKFTargetStateEstimation::correctionStep(sensor_model::SensorMeasurement const& measurement,
                                              math::VehicleState const& interceptor_state)
{
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
    return MeasurementJacobian{};
}

}  // namespace estimation