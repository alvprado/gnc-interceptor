#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "guidance/parameters/controller_config.hpp"

namespace
{

class ControllerConfigTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    static gnc_ros::ControllerConfig read(std::vector<rclcpp::Parameter> const& parameters)
    {
        rclcpp::NodeOptions options;
        options.parameter_overrides(parameters);
        rclcpp::Node node{"controller_config_test", options};
        return gnc_ros::readControllerConfig(node);
    }
};

TEST_F(ControllerConfigTest, DefaultsMatchStandaloneProportionalNavigation)
{
    auto const config = read({});
    EXPECT_EQ(config.type, gnc_ros::ControllerType::ProportionalNavigation);
    EXPECT_DOUBLE_EQ(config.vehicle.mass_kg, 2.5);
    EXPECT_DOUBLE_EQ(config.min_load_factor, -3.0);
    EXPECT_DOUBLE_EQ(config.max_load_factor, 9.0);
    EXPECT_DOUBLE_EQ(config.boost_phase_switch_speed_mps, 90.0);
    EXPECT_DOUBLE_EQ(config.boost_phase_thrust_n, 150.0);
    EXPECT_DOUBLE_EQ(config.navigation_gain, 3.0);

    auto controller = gnc_ros::makeGuidanceController(config);
    math::CartesianState const target{{3000.0, 0.0, 0.0}, {-10.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    math::CartesianState const interceptor{{0.0, 0.0, 0.0}, {100.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    auto const control = controller.step(target, interceptor, 0.01);
    EXPECT_TRUE(control.allFinite());
}

TEST_F(ControllerConfigTest, ReadsPredictive)
{
    auto const config = read({
        {"controller.type", "predictive"},
        {"controller.horizon", 10},
        {"controller.min_horizon", 2},
        {"controller.dt", 0.2},
        {"controller.final_interception_weight", 5.0},
        {"controller.running_interception_weight", 1.0},
    });
    EXPECT_EQ(config.type, gnc_ros::ControllerType::Predictive);
    EXPECT_EQ(config.horizon, 10);
    EXPECT_EQ(config.min_horizon, 2);
    EXPECT_DOUBLE_EQ(config.dt, 0.2);
    EXPECT_DOUBLE_EQ(config.final_interception_weight, 5.0);
    EXPECT_DOUBLE_EQ(config.running_interception_weight, 1.0);

    auto controller = gnc_ros::makeGuidanceController(config);
    math::CartesianState const target{{3000.0, 0.0, 0.0}, {-10.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    math::CartesianState const interceptor{{0.0, 0.0, 0.0}, {100.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    auto const control = controller.step(target, interceptor, 0.01);
    EXPECT_TRUE(control.allFinite());
}

TEST_F(ControllerConfigTest, ParametersAreReadOnlyAndSpecificToSelectedType)
{
    rclcpp::NodeOptions options;
    options.parameter_overrides({{"controller.type", "proportional_navigation"}});
    rclcpp::Node node{"controller_config_test", options};
    (void)gnc_ros::readControllerConfig(node);
    EXPECT_FALSE(node.has_parameter("controller.horizon"));
    EXPECT_FALSE(node.set_parameter({"controller.type", "predictive"}).successful);
    EXPECT_FALSE(node.set_parameter({"controller.navigation_gain", 5.0}).successful);
}

}  // namespace
