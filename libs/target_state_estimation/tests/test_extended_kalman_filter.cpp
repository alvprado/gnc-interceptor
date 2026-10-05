#include "estimation/extended_kalman_filter.hpp"

#include "math/angles.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace estimation
{
namespace
{

constexpr double k_tol{1.0e-9};

void expectStateNear(math::CartesianState const& actual, math::CartesianState const& expected,
                     double tolerance = k_tol)
{
    EXPECT_LE((actual.position_m - expected.position_m).norm(), tolerance);
    EXPECT_LE((actual.velocity_mps - expected.velocity_mps).norm(), tolerance);
    EXPECT_LE((actual.acceleration_mps2 - expected.acceleration_mps2).norm(), tolerance);
}

class EKFTest : public ::testing::Test
{
protected:
    EKFTargetStateEstimationConfig config_{};
    EKFTargetStateEstimation filter_{config_};
    math::VehicleState observer_{};
    sensor_model::SensorMeasurement measurement_{math::Timestamp{5.0}, 100.0, 0.0, 0.0, 0.0};
};

TEST_F(EKFTest, InitializesPositionFromRangeAndBodyAnglesAtObserverPose)
{
    observer_.cartesian.position_m = Eigen::Vector3d{20.0, -40.0, 10.0};
    observer_.attitude = math::attitudeFromHeadingPitchBank(0.7, -0.3, 0.2);
    // A 3-4-5 triangle in the body xy plane, with a 12 m vertical component.
    measurement_.range_m = 13.0;
    measurement_.azimuth_rad = std::atan2(4.0, 3.0);
    measurement_.elevation_rad = std::atan2(12.0, 5.0);
    filter_.processMeasurement(measurement_, observer_);

    auto const output = filter_.stateEstimateAt(measurement_.timestamp);
    math::CartesianState expected;
    expected.position_m = observer_.cartesian.position_m +
                          observer_.attitude * Eigen::Vector3d{3.0, 4.0, 12.0};
    EXPECT_EQ(output.filter_status, EKFStatus::Valid);
    expectStateNear(output.target_state_estimate, expected);
}

TEST_F(EKFTest, InitializesConfiguredCovarianceWithoutCrossCorrelations)
{
    config_.initial_position_var = 9.0;
    config_.initial_velocity_var = 16.0;
    config_.initial_acceleration_var = 25.0;
    EKFTargetStateEstimation filter{config_};
    filter.processMeasurement(measurement_, observer_);

    StateVec diagonal;
    diagonal << 9.0, 9.0, 9.0, 16.0, 16.0, 16.0, 25.0, 25.0, 25.0;
    StateCov const expected = diagonal.asDiagonal();
    EXPECT_TRUE(filter.errorCovarianceAt(measurement_.timestamp).isApprox(expected, k_tol));
}

TEST_F(EKFTest, SameTimestampCorrectionMatchesScalarRangeAndDopplerUpdates)
{
    observer_.cartesian.velocity_mps.x() = 7.0;
    filter_.processMeasurement(measurement_, observer_);
    measurement_.range_m = 110.0;
    measurement_.range_rate_mps = 3.0;
    filter_.processMeasurement(measurement_, observer_);

    double const position_gain = config_.initial_position_var /
                                 (config_.initial_position_var + config_.sensor_noise_var[0]);
    double const velocity_gain = config_.initial_velocity_var /
                                 (config_.initial_velocity_var + config_.sensor_noise_var[1]);
    math::CartesianState expected;
    expected.position_m.x() = 100.0 + position_gain * 10.0;
    expected.velocity_mps.x() = velocity_gain * 10.0;
    auto const output = filter_.stateEstimateAt(measurement_.timestamp);
    EXPECT_EQ(output.filter_status, EKFStatus::Valid);
    expectStateNear(output.target_state_estimate, expected);

    auto const covariance = filter_.errorCovarianceAt(measurement_.timestamp);
    EXPECT_NEAR(covariance(0, 0), (1.0 - position_gain) * config_.initial_position_var, k_tol);
    EXPECT_NEAR(covariance(3, 3), (1.0 - velocity_gain) * config_.initial_velocity_var, k_tol);
}

TEST_F(EKFTest, ZeroInnovationPreservesStateAndReducesUncertainty)
{
    filter_.processMeasurement(measurement_, observer_);
    auto const before = filter_.stateEstimateAt(measurement_.timestamp);
    auto const covariance_before = filter_.errorCovarianceAt(measurement_.timestamp);
    filter_.processMeasurement(measurement_, observer_);

    expectStateNear(filter_.stateEstimateAt(measurement_.timestamp).target_state_estimate,
                    before.target_state_estimate);
    auto const covariance_after = filter_.errorCovarianceAt(measurement_.timestamp);
    for (int i = 0; i < 4; ++i)
    {
        EXPECT_LT(covariance_after(i, i), covariance_before(i, i));
    }
}

TEST_F(EKFTest, AzimuthInnovationCrossesBranchCutInBothDirections)
{
    for (double const direction : {-1.0, 1.0})
    {
        EKFTargetStateEstimation filter{config_};
        measurement_.azimuth_rad = direction * (std::acos(-1.0) - 0.001);
        filter.processMeasurement(measurement_, observer_);
        measurement_.azimuth_rad = -measurement_.azimuth_rad;
        filter.processMeasurement(measurement_, observer_);

        auto const output = filter.stateEstimateAt(measurement_.timestamp);
        EXPECT_EQ(output.filter_status, EKFStatus::Valid);
        EXPECT_NEAR(output.target_state_estimate.position_m.x(), -100.0, 0.001);
        EXPECT_NEAR(output.target_state_estimate.position_m.y(), -direction * 0.1, 0.001);
        EXPECT_LT(output.target_state_estimate.velocity_mps.norm(), 1.0e-9);
    }
}

TEST_F(EKFTest, FutureQueryExtrapolatesConstantAccelerationWithoutChangingTrack)
{
    filter_.processMeasurement(measurement_, observer_);
    measurement_.timestamp += math::Timestamp{1.0};
    measurement_.range_m = 112.0;
    measurement_.range_rate_mps = 12.0;
    filter_.processMeasurement(measurement_, observer_);
    auto const current = filter_.stateEstimateAt(measurement_.timestamp);
    auto const covariance = filter_.errorCovarianceAt(measurement_.timestamp);
    ASSERT_GT(current.target_state_estimate.velocity_mps.norm(), 1.0);
    ASSERT_GT(current.target_state_estimate.acceleration_mps2.norm(), 0.01);

    constexpr double dt{2.5};
    auto expected = current.target_state_estimate;
    expected.position_m += dt * expected.velocity_mps +
                           0.5 * dt * dt * expected.acceleration_mps2;
    expected.velocity_mps += dt * expected.acceleration_mps2;
    auto const future = filter_.stateEstimateAt(measurement_.timestamp + math::Timestamp{dt});
    EXPECT_EQ(future.filter_status, EKFStatus::Valid);
    expectStateNear(future.target_state_estimate, expected);
    EXPECT_GT(filter_.errorCovarianceAt(measurement_.timestamp + math::Timestamp{dt}).trace(),
              covariance.trace());
    expectStateNear(filter_.stateEstimateAt(measurement_.timestamp).target_state_estimate,
                    current.target_state_estimate);
    EXPECT_TRUE(filter_.errorCovarianceAt(measurement_.timestamp).isApprox(covariance, k_tol));
}

TEST_F(EKFTest, CovariancePredictionMatchesIntegratedWhiteJerkAtOneSecond)
{
    config_.initial_position_var = 2.0;
    config_.initial_velocity_var = 3.0;
    config_.initial_acceleration_var = 4.0;
    config_.jerk_noise_var = 120.0;
    EKFTargetStateEstimation filter{config_};
    filter.processMeasurement(measurement_, observer_);

    // One-axis [position, velocity, acceleration] covariance at dt = 1 s.
    Eigen::Matrix3d axis_covariance;
    axis_covariance << 12.0, 20.0, 22.0,
                       20.0, 47.0, 64.0,
                       22.0, 64.0, 124.0;
    StateCov expected = StateCov::Zero();
    for (int row = 0; row < 3; ++row)
    {
        for (int col = 0; col < 3; ++col)
        {
            expected.block<3, 3>(3 * row, 3 * col) =
                axis_covariance(row, col) * Eigen::Matrix3d::Identity();
        }
    }
    EXPECT_TRUE(filter.errorCovarianceAt(measurement_.timestamp + math::Timestamp{1.0})
                    .isApprox(expected, k_tol));
}

TEST_F(EKFTest, StaleQueryReturnsCurrentStateAndCovarianceWithoutChangingStatus)
{
    filter_.processMeasurement(measurement_, observer_);
    auto const current = filter_.stateEstimateAt(measurement_.timestamp);
    auto const covariance = filter_.errorCovarianceAt(measurement_.timestamp);
    auto const stale_time = measurement_.timestamp - math::Timestamp{1.0};

    auto const stale = filter_.stateEstimateAt(stale_time);
    EXPECT_EQ(stale.filter_status, EKFStatus::StaleTimestamp);
    expectStateNear(stale.target_state_estimate, current.target_state_estimate);
    EXPECT_TRUE(filter_.errorCovarianceAt(stale_time).isApprox(covariance, k_tol));
    EXPECT_EQ(filter_.stateEstimateAt(measurement_.timestamp).filter_status, EKFStatus::Valid);
}

TEST_F(EKFTest, RejectsStaleMeasurementAndRecoversOnNextUpdate)
{
    filter_.processMeasurement(measurement_, observer_);
    auto const current = filter_.stateEstimateAt(measurement_.timestamp);
    auto const covariance = filter_.errorCovarianceAt(measurement_.timestamp);
    auto stale = measurement_;
    stale.timestamp -= math::Timestamp{1.0};
    stale.range_m = 900.0;
    filter_.processMeasurement(stale, observer_);

    auto const rejected = filter_.stateEstimateAt(measurement_.timestamp);
    EXPECT_EQ(rejected.filter_status, EKFStatus::StaleTimestamp);
    expectStateNear(rejected.target_state_estimate, current.target_state_estimate);
    EXPECT_TRUE(filter_.errorCovarianceAt(measurement_.timestamp).isApprox(covariance, k_tol));

    measurement_.timestamp += math::Timestamp{0.5};
    filter_.processMeasurement(measurement_, observer_);
    EXPECT_EQ(filter_.stateEstimateAt(measurement_.timestamp).filter_status, EKFStatus::Valid);
    expectStateNear(filter_.stateEstimateAt(measurement_.timestamp).target_state_estimate,
                    current.target_state_estimate);
}

TEST_F(EKFTest, RepeatedCoincidentMeasurementsRemainFinite)
{
    measurement_.range_m = 0.0;
    for (int i = 0; i < 10; ++i)
    {
        measurement_.timestamp = math::Timestamp{0.1 * i};
        filter_.processMeasurement(measurement_, observer_);
        auto const output = filter_.stateEstimateAt(measurement_.timestamp);
        EXPECT_EQ(output.filter_status, EKFStatus::Valid);
        expectStateNear(output.target_state_estimate, math::CartesianState{});
        EXPECT_TRUE(filter_.errorCovarianceAt(measurement_.timestamp).allFinite());
    }
}

TEST_F(EKFTest, TracksConstantAccelerationWithMovingRotatedObserver)
{
    sensor_model::RadarModel radar{sensor_model::RadarModelConfig{0.0, 0.0, 0.0, 0.0}, 42};
    observer_.attitude = math::attitudeFromHeadingPitchBank(0.4, -0.2, 0.1);
    observer_.cartesian.velocity_mps = Eigen::Vector3d{3.0, -2.0, 1.0};
    Eigen::Vector3d const initial_position{1000.0, 300.0, 200.0};
    Eigen::Vector3d const initial_velocity{10.0, 5.0, -2.0};
    Eigen::Vector3d const acceleration{0.2, -0.1, 0.05};
    for (int i = 0; i <= 300; ++i)
    {
        SCOPED_TRACE(i);
        double const t = 0.1 * i;
        observer_.cartesian.position_m = t * observer_.cartesian.velocity_mps;
        math::CartesianState const truth{
            initial_position + t * initial_velocity + 0.5 * t * t * acceleration,
            initial_velocity + t * acceleration, acceleration};
        auto const measurement = radar.step(truth, observer_, math::Timestamp{t});
        filter_.processMeasurement(measurement, observer_);
        auto const output = filter_.stateEstimateAt(measurement.timestamp);
        ASSERT_EQ(output.filter_status, EKFStatus::Valid);
        auto const covariance = filter_.errorCovarianceAt(measurement.timestamp);
        ASSERT_TRUE(covariance.allFinite());
        EXPECT_TRUE(covariance.isApprox(covariance.transpose(), 1.0e-10));
        Eigen::SelfAdjointEigenSolver<StateCov> const eigensolver{covariance};
        ASSERT_EQ(eigensolver.info(), Eigen::Success);
        EXPECT_GE(eigensolver.eigenvalues().minCoeff(), -1.0e-9);
        if (i >= 200)
        {
            EXPECT_LT((output.target_state_estimate.position_m - truth.position_m).norm(), 0.1);
            EXPECT_LT((output.target_state_estimate.velocity_mps - truth.velocity_mps).norm(), 0.1);
            EXPECT_LT((output.target_state_estimate.acceleration_mps2 - acceleration).norm(), 0.1);
        }
    }
}

}  // namespace
}  // namespace estimation
