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
    return StateJacobian{};
}

StateCov EKFTargetStateEstimation::processNoiseCovariance(double dt) const { return StateCov{}; }

MeasurementJacobian EKFTargetStateEstimation::measurementJacobian(
    StateVec const& state, math::VehicleState const& interceptor_state) const
{
    return MeasurementJacobian{};
}

}  // namespace estimation