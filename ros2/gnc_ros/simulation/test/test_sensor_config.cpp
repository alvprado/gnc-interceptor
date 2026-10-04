#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "simulation/parameters/sensor_config.hpp"

namespace
{

class SensorConfigTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    static sensor_model::RadarModelConfig read(std::vector<rclcpp::Parameter> const& parameters)
    {
        rclcpp::NodeOptions options;
        options.parameter_overrides(parameters);
        rclcpp::Node node{"sensor_config_test", options};
        return gnc_ros::readSensorConfig(node);
    }
};

TEST_F(SensorConfigTest, DefaultsMatchStandaloneRadarModelConfig)
{
    auto const config = read({});
    EXPECT_DOUBLE_EQ(config.range_var, sensor_model::RadarModelConfig{}.range_var);
    EXPECT_DOUBLE_EQ(config.range_rate_var, sensor_model::RadarModelConfig{}.range_rate_var);
    EXPECT_DOUBLE_EQ(config.azimuth_var, sensor_model::RadarModelConfig{}.azimuth_var);
    EXPECT_DOUBLE_EQ(config.elevation_var, sensor_model::RadarModelConfig{}.elevation_var);
}

TEST_F(SensorConfigTest, ReadsOverriddenVariances)
{
    auto const config = read({
        {"sensor.range_var", 100.0},
        {"sensor.range_rate_var", 4.0},
        {"sensor.azimuth_var", 1.0e-5},
        {"sensor.elevation_var", 2.0e-5},
    });
    EXPECT_DOUBLE_EQ(config.range_var, 100.0);
    EXPECT_DOUBLE_EQ(config.range_rate_var, 4.0);
    EXPECT_DOUBLE_EQ(config.azimuth_var, 1.0e-5);
    EXPECT_DOUBLE_EQ(config.elevation_var, 2.0e-5);

    // Construction should not throw given a valid configuration.
    (void)gnc_ros::makeSensorModel(config);
}

TEST_F(SensorConfigTest, ParametersAreReadOnly)
{
    rclcpp::NodeOptions options;
    rclcpp::Node node{"sensor_config_test", options};
    (void)gnc_ros::readSensorConfig(node);
    EXPECT_FALSE(node.set_parameter({"sensor.range_var", 10.0}).successful);
}

}  // namespace
