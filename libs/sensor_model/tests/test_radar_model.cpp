#include "sensor_model/radar_model.hpp"

#include "math/angles.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

namespace sensor_model
{
namespace
{

constexpr double k_tol{1.0e-6};
constexpr Timestamp k_timestamp{0.0};

/// @brief A RadarModelConfig with (near-)zero noise, for exercising the
/// noise-free geometry: variances of 0 are clamped internally to a tiny
/// floor, well within k_tol.
constexpr RadarModelConfig k_noiseless_config{0.0, 0.0, 0.0, 0.0};

[[nodiscard]] math::CartesianState makeCartesian(Eigen::Vector3d const& position_m,
                                                  Eigen::Vector3d const& velocity_mps)
{
    math::CartesianState state;
    state.position_m = position_m;
    state.velocity_mps = velocity_mps;
    return state;
}

[[nodiscard]] math::VehicleState makeVehicle(Eigen::Vector3d const& position_m,
                                             Eigen::Vector3d const& velocity_mps,
                                             Eigen::Quaterniond const& orientation =
                                                 Eigen::Quaterniond::Identity())
{
    return math::VehicleState{makeCartesian(position_m, velocity_mps), orientation};
}

TEST(RadarModelTest, RangeMatchesEuclideanDistance)
{
    RadarModel radar{k_noiseless_config, 1};
    auto const target = makeCartesian(Eigen::Vector3d{30.0, 40.0, 0.0}, Eigen::Vector3d::Zero());
    auto const interceptor = makeVehicle(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    auto const measurement = radar.step(target, interceptor, k_timestamp);

    EXPECT_NEAR(measurement.range_m, 50.0, k_tol);
}

TEST(RadarModelTest, TargetDirectlyAheadGivesZeroAzimuthAndElevation)
{
    RadarModel radar{k_noiseless_config, 1};
    auto const target = makeCartesian(Eigen::Vector3d{100.0, 0.0, 0.0}, Eigen::Vector3d::Zero());
    auto const interceptor = makeVehicle(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    auto const measurement = radar.step(target, interceptor, k_timestamp);

    EXPECT_NEAR(measurement.azimuth_rad, 0.0, k_tol);
    EXPECT_NEAR(measurement.elevation_rad, 0.0, k_tol);
}

TEST(RadarModelTest, TargetAboveGivesNinetyDegreeElevation)
{
    RadarModel radar{k_noiseless_config, 1};
    auto const target = makeCartesian(Eigen::Vector3d{0.0, 0.0, 100.0}, Eigen::Vector3d::Zero());
    auto const interceptor = makeVehicle(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    auto const measurement = radar.step(target, interceptor, k_timestamp);

    EXPECT_NEAR(measurement.elevation_rad, std::numbers::pi / 2.0, k_tol);
}

TEST(RadarModelTest, RangeRateIsPositiveWhenSeparating)
{
    RadarModel radar{k_noiseless_config, 1};
    auto const target =
        makeCartesian(Eigen::Vector3d{100.0, 0.0, 0.0}, Eigen::Vector3d{10.0, 0.0, 0.0});
    auto const interceptor = makeVehicle(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    auto const measurement = radar.step(target, interceptor, k_timestamp);

    EXPECT_NEAR(measurement.range_rate_mps, 10.0, k_tol);
}

TEST(RadarModelTest, RangeRateIsNegativeWhenClosing)
{
    RadarModel radar{k_noiseless_config, 1};
    auto const target =
        makeCartesian(Eigen::Vector3d{100.0, 0.0, 0.0}, Eigen::Vector3d{-10.0, 0.0, 0.0});
    auto const interceptor = makeVehicle(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    auto const measurement = radar.step(target, interceptor, k_timestamp);

    EXPECT_NEAR(measurement.range_rate_mps, -10.0, k_tol);
}

TEST(RadarModelTest, RangeRateStaysFiniteAtZeroRange)
{
    RadarModel radar{k_noiseless_config, 1};
    auto const target = makeCartesian(Eigen::Vector3d::Zero(), Eigen::Vector3d{5.0, 0.0, 0.0});
    auto const interceptor = makeVehicle(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    auto const measurement = radar.step(target, interceptor, k_timestamp);

    EXPECT_TRUE(std::isfinite(measurement.range_rate_mps));
}

TEST(RadarModelTest, AzimuthAccountsForInterceptorYaw)
{
    RadarModel radar{k_noiseless_config, 1};
    // Interceptor yawed +90deg: body forward now points along world +y.
    auto const interceptor_orientation =
        math::attitudeFromHeadingPitchBank(std::numbers::pi / 2.0, 0.0, 0.0);
    auto const interceptor =
        makeVehicle(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), interceptor_orientation);
    // Target is along world +x, i.e. 90deg to the interceptor's right.
    auto const target = makeCartesian(Eigen::Vector3d{100.0, 0.0, 0.0}, Eigen::Vector3d::Zero());

    auto const measurement = radar.step(target, interceptor, k_timestamp);

    EXPECT_NEAR(measurement.azimuth_rad, -std::numbers::pi / 2.0, k_tol);
}

TEST(RadarModelTest, TimestampIsCarriedThroughUnchanged)
{
    RadarModel radar{k_noiseless_config, 1};
    auto const target = makeCartesian(Eigen::Vector3d{100.0, 0.0, 0.0}, Eigen::Vector3d::Zero());
    auto const interceptor = makeVehicle(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    Timestamp const timestamp{12.5};

    auto const measurement = radar.step(target, interceptor, timestamp);

    EXPECT_EQ(measurement.timestamp, timestamp);
}

TEST(RadarModelTest, SameSeedReproducesTheSameMeasurement)
{
    RadarModelConfig const config{4.0, 4.0, 0.01, 0.01};
    RadarModel radar_a{config, 123};
    RadarModel radar_b{config, 123};
    auto const target = makeCartesian(Eigen::Vector3d{100.0, 20.0, 5.0}, Eigen::Vector3d{1, 2, 3});
    auto const interceptor = makeVehicle(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    auto const a = radar_a.step(target, interceptor, k_timestamp);
    auto const b = radar_b.step(target, interceptor, k_timestamp);

    EXPECT_DOUBLE_EQ(a.range_m, b.range_m);
    EXPECT_DOUBLE_EQ(a.range_rate_mps, b.range_rate_mps);
    EXPECT_DOUBLE_EQ(a.azimuth_rad, b.azimuth_rad);
    EXPECT_DOUBLE_EQ(a.elevation_rad, b.elevation_rad);
}

TEST(RadarModelTest, DifferentSeedsGiveDifferentMeasurements)
{
    RadarModelConfig const config{4.0, 4.0, 0.01, 0.01};
    RadarModel radar_a{config, 123};
    RadarModel radar_b{config, 456};
    auto const target = makeCartesian(Eigen::Vector3d{100.0, 20.0, 5.0}, Eigen::Vector3d{1, 2, 3});
    auto const interceptor = makeVehicle(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    auto const a = radar_a.step(target, interceptor, k_timestamp);
    auto const b = radar_b.step(target, interceptor, k_timestamp);

    EXPECT_NE(a.range_m, b.range_m);
}

TEST(RadarModelTest, NoiseChannelsAreIndependentAcrossManySamples)
{
    // If the four channels shared RNG state (the original copy bug this
    // class was fixed to avoid), their noise would be perfectly correlated.
    RadarModelConfig const config{4.0, 4.0, 4.0, 4.0};
    RadarModel radar{config, 99};
    auto const target = makeCartesian(Eigen::Vector3d{100.0, 0.0, 0.0}, Eigen::Vector3d::Zero());
    auto const interceptor = makeVehicle(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    double sum_range_az{0.0};
    double sum_range{0.0}, sum_sq_range{0.0};
    double sum_az{0.0}, sum_sq_az{0.0};
    constexpr int n{20000};
    for (int i = 0; i < n; ++i)
    {
        auto const m = radar.step(target, interceptor, k_timestamp);
        sum_range += m.range_m;
        sum_sq_range += m.range_m * m.range_m;
        sum_az += m.azimuth_rad;
        sum_sq_az += m.azimuth_rad * m.azimuth_rad;
        sum_range_az += m.range_m * m.azimuth_rad;
    }

    double const mean_range = sum_range / n;
    double const mean_az = sum_az / n;
    double const cov = sum_range_az / n - mean_range * mean_az;
    double const var_range = sum_sq_range / n - mean_range * mean_range;
    double const var_az = sum_sq_az / n - mean_az * mean_az;
    double const correlation = cov / std::sqrt(var_range * var_az);

    EXPECT_NEAR(correlation, 0.0, 0.05) << "range and azimuth noise must be independent";
}

}  // namespace
}  // namespace sensor_model
