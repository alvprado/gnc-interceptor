#pragma once

#include <Eigen/Dense>

#include "math/state_types.hpp"
#include "sensor_model/radar_model.hpp"

namespace estimation
{

using StateVec = Eigen::Vector<double, 9>;
using StateCov = Eigen::Matrix<double, 9, 9>;
using StateJacobian = StateCov;
using MeasurementVec = Eigen::Vector<double, 4>;
using MeasurementCov = Eigen::Matrix<double, 4, 4>;
using MeasurementJacobian = Eigen::Matrix<double, 4, 9>;

struct EKFTargetStateEstimationConfig
{
    double jerk_noise_var{1.0};
    MeasurementVec sensor_noise_var{1.0, 1.0, 1.0, 1.0};
};

class EKFTargetStateEstimation
{
public:
    explicit EKFTargetStateEstimation(EKFTargetStateEstimationConfig const& config);

    [[nodiscard]] math::CartesianState estimateAt(math::Timestamp timestamp) const;

    void processMeasurement(sensor_model::SensorMeasurement const& measurement,
                            math::VehicleState const& interceptor_state);

private:
    void predictionStep(math::Timestamp timestamp);

    void correctionStep(sensor_model::SensorMeasurement const& measurement,
                        math::VehicleState const& interceptor_state);

    [[nodiscard]] StateJacobian stateTransitionMatrix(double dt) const;

    [[nodiscard]] StateCov processNoiseCovariance(double dt) const;

    [[nodiscard]] MeasurementJacobian measurementJacobian(
        StateVec const& state, math::VehicleState const& interceptor_state) const;

    double jerk_noise_var_;
    MeasurementCov measurement_noise_cov_;

    math::Timestamp filter_time_{0.0};
    StateVec state_estimate_{};
};
}  // namespace estimation