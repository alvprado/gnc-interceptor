#include <gtest/gtest.h>

#include "common/converters.hpp"

namespace
{

TEST(ConvertersTest, TimestampsRoundToNanosecondsAndNormalizeSeconds)
{
    auto const zero = gnc_ros::toMsg(math::Timestamp{0.0});
    EXPECT_EQ(zero.sec, 0);
    EXPECT_EQ(zero.nanosec, 0U);
    auto const stamp = gnc_ros::toMsg(math::Timestamp{12.345678901});
    EXPECT_EQ(stamp.sec, 12);
    EXPECT_EQ(stamp.nanosec, 345678901U);
    EXPECT_NEAR(gnc_ros::fromMsg(stamp).count(), 12.345678901, 1.0e-12);
    auto const carry = gnc_ros::toMsg(math::Timestamp{0.9999999996});
    EXPECT_EQ(carry.sec, 1);
    EXPECT_EQ(carry.nanosec, 0U);
}

TEST(ConvertersTest, VehicleStatePreservesComponentsAndQuaternionOrder)
{
    math::VehicleState const state{
        {{1.0, -2.0, 3.0}, {4.0, -5.0, 6.0}, {7.0, -8.0, 9.0}},
        Eigen::Quaterniond{4.0, 1.0, 2.0, 3.0}.normalized()};
    auto const stamp = gnc_ros::toMsg(math::Timestamp{1.25});
    auto const message = gnc_ros::toMsg(state, stamp, "world");
    EXPECT_EQ(message.header.stamp, stamp);
    EXPECT_EQ(message.header.frame_id, "world");
    EXPECT_DOUBLE_EQ(message.position_m.y, -2.0);
    EXPECT_DOUBLE_EQ(message.velocity_mps.z, 6.0);
    EXPECT_DOUBLE_EQ(message.acceleration_mps2.x, 7.0);
    EXPECT_DOUBLE_EQ(message.attitude.w, state.attitude.w());
    EXPECT_DOUBLE_EQ(message.attitude.x, state.attitude.x());
    EXPECT_DOUBLE_EQ(message.attitude.y, state.attitude.y());
    EXPECT_DOUBLE_EQ(message.attitude.z, state.attitude.z());
    auto const restored = gnc_ros::fromMsg(message);
    EXPECT_TRUE(restored.cartesian.position_m.isApprox(state.cartesian.position_m));
    EXPECT_TRUE(restored.cartesian.velocity_mps.isApprox(state.cartesian.velocity_mps));
    EXPECT_TRUE(restored.cartesian.acceleration_mps2.isApprox(state.cartesian.acceleration_mps2));
    EXPECT_TRUE(restored.attitude.coeffs().isApprox(state.attitude.coeffs()));
}

TEST(ConvertersTest, TargetStatePreservesAllCartesianComponents)
{
    math::CartesianState const state{{-1.0, 2.0, 3.0}, {4.0, 5.0, -6.0}, {-7.0, 8.0, 9.0}};
    auto const stamp = gnc_ros::toMsg(math::Timestamp{2.5});
    auto const message = gnc_ros::toMsg(state, stamp, "world");
    EXPECT_EQ(message.header.stamp, stamp);
    EXPECT_EQ(message.header.frame_id, "world");
    auto const restored = gnc_ros::fromMsg(message);
    EXPECT_TRUE(restored.position_m.isApprox(state.position_m));
    EXPECT_TRUE(restored.velocity_mps.isApprox(state.velocity_mps));
    EXPECT_TRUE(restored.acceleration_mps2.isApprox(state.acceleration_mps2));
}

TEST(ConvertersTest, RadarSamplePreservesTimestampAndMeasurementChannels)
{
    sensor_model::SensorMeasurement const sample{math::Timestamp{2.75}, 100.0, -20.0, 0.3, -0.4};
    auto const message = gnc_ros::toMsg(sample, "interceptor_body");
    EXPECT_EQ(message.header.stamp, gnc_ros::toMsg(sample.timestamp));
    EXPECT_EQ(message.header.frame_id, "interceptor_body");
    auto const restored = gnc_ros::fromMsg(message);
    EXPECT_DOUBLE_EQ(restored.timestamp.count(), 2.75);
    EXPECT_DOUBLE_EQ(restored.range_m, 100.0);
    EXPECT_DOUBLE_EQ(restored.range_rate_mps, -20.0);
    EXPECT_DOUBLE_EQ(restored.azimuth_rad, 0.3);
    EXPECT_DOUBLE_EQ(restored.elevation_rad, -0.4);
}

TEST(ConvertersTest, GuidanceCommandUsesThrustLoadBankOrder)
{
    simulation::UAV3DofModel::ControlVec const control{50.0, 2.0, -0.3};
    auto const stamp = gnc_ros::toMsg(math::Timestamp{3.0});
    auto const message = gnc_ros::toMsg(control, stamp, "interceptor_body");
    EXPECT_EQ(message.header.stamp, stamp);
    EXPECT_EQ(message.header.frame_id, "interceptor_body");
    EXPECT_DOUBLE_EQ(message.thrust_n, 50.0);
    EXPECT_DOUBLE_EQ(message.load_factor, 2.0);
    EXPECT_DOUBLE_EQ(message.bank_angle_rad, -0.3);
    EXPECT_TRUE(gnc_ros::fromMsg(message).isApprox(control));
}

}  // namespace
