#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "estimation/parameters/ekf_config.hpp"

namespace
{

class EkfConfigTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    static estimation::EKFTargetStateEstimationConfig read(
        std::vector<rclcpp::Parameter> const& parameters)
    {
        rclcpp::NodeOptions options;
        options.parameter_overrides(parameters);
        rclcpp::Node node{"ekf_config_test", options};
        return gnc_ros::readEkfConfig(node);
    }
};

TEST_F(EkfConfigTest, DefaultsMatchStandaloneEkf)
{
    auto const config = read({});
    EXPECT_DOUBLE_EQ(config.jerk_noise_var, 10.0);
    EXPECT_TRUE(
        config.sensor_noise_var.isApprox(estimation::MeasurementVec{100.0, 4.0, 4.0e-6, 4.0e-6}));
    EXPECT_DOUBLE_EQ(config.initial_position_var, 1.0e4);
    EXPECT_DOUBLE_EQ(config.initial_velocity_var, 4.0e4);
    EXPECT_DOUBLE_EQ(config.initial_acceleration_var, 4.0e2);
}

TEST_F(EkfConfigTest, ReadsOverriddenValues)
{
    auto const config = read({
        {"ekf.jerk_noise_var", 5.0},
        {"ekf.sensor_noise_var", std::vector<double>{1.0, 2.0, 3.0, 4.0}},
        {"ekf.initial_position_var", 1.0},
        {"ekf.initial_velocity_var", 2.0},
        {"ekf.initial_acceleration_var", 3.0},
    });
    EXPECT_DOUBLE_EQ(config.jerk_noise_var, 5.0);
    EXPECT_TRUE(config.sensor_noise_var.isApprox(estimation::MeasurementVec{1.0, 2.0, 3.0, 4.0}));
    EXPECT_DOUBLE_EQ(config.initial_position_var, 1.0);
    EXPECT_DOUBLE_EQ(config.initial_velocity_var, 2.0);
    EXPECT_DOUBLE_EQ(config.initial_acceleration_var, 3.0);
}

TEST_F(EkfConfigTest, ParametersAreReadOnly)
{
    rclcpp::NodeOptions options;
    rclcpp::Node node{"ekf_config_test", options};
    (void)gnc_ros::readEkfConfig(node);
    EXPECT_FALSE(node.set_parameter({"ekf.jerk_noise_var", 99.0}).successful);
}

}  // namespace
