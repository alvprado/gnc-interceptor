#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "simulation/parameters/interceptor_config.hpp"

namespace
{

class InterceptorConfigTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    static gnc_ros::InterceptorConfig read(std::vector<rclcpp::Parameter> const& parameters)
    {
        rclcpp::NodeOptions options;
        options.parameter_overrides(parameters);
        rclcpp::Node node{"interceptor_config_test", options};
        return gnc_ros::readInterceptorConfig(node);
    }
};

TEST_F(InterceptorConfigTest, DefaultsMatchStandaloneParamsAndLimits)
{
    auto const config = read({});
    EXPECT_DOUBLE_EQ(config.params.mass_kg, simulation::UAV3DofModelParams{}.mass_kg);
    EXPECT_DOUBLE_EQ(config.params.rho_kgpm3, simulation::UAV3DofModelParams{}.rho_kgpm3);
    EXPECT_DOUBLE_EQ(config.params.frontal_area_m2, simulation::UAV3DofModelParams{}.frontal_area_m2);
    EXPECT_DOUBLE_EQ(config.params.drag_coeff, simulation::UAV3DofModelParams{}.drag_coeff);

    EXPECT_DOUBLE_EQ(config.limits.min_thrust_n, simulation::UAV3DofModelLimits{}.min_thrust_n);
    EXPECT_DOUBLE_EQ(config.limits.max_thrust_n, simulation::UAV3DofModelLimits{}.max_thrust_n);
    EXPECT_DOUBLE_EQ(config.limits.min_load_factor, simulation::UAV3DofModelLimits{}.min_load_factor);
    EXPECT_DOUBLE_EQ(config.limits.max_load_factor, simulation::UAV3DofModelLimits{}.max_load_factor);
    EXPECT_DOUBLE_EQ(config.limits.min_speed_mps, simulation::UAV3DofModelLimits{}.min_speed_mps);
    EXPECT_DOUBLE_EQ(config.limits.max_speed_mps, simulation::UAV3DofModelLimits{}.max_speed_mps);
}

TEST_F(InterceptorConfigTest, ReadsOverriddenParamsAndLimits)
{
    auto const config = read({
        {"interceptor.mass_kg", 5.0},
        {"interceptor.drag_coeff", 0.2},
        {"interceptor.max_thrust_n", 300.0},
        {"interceptor.min_speed_mps", 2.0},
        {"interceptor.max_speed_mps", 200.0},
    });
    EXPECT_DOUBLE_EQ(config.params.mass_kg, 5.0);
    EXPECT_DOUBLE_EQ(config.params.drag_coeff, 0.2);
    EXPECT_DOUBLE_EQ(config.limits.max_thrust_n, 300.0);
    EXPECT_DOUBLE_EQ(config.limits.min_speed_mps, 2.0);
    EXPECT_DOUBLE_EQ(config.limits.max_speed_mps, 200.0);

    auto const model = gnc_ros::makeInterceptorModel(config);
    EXPECT_TRUE(model.clampState(simulation::UAV3DofModel::StateVec{0.0, 0.0, 0.0, 1000.0, 0.0, 0.0})
                    .isApprox(simulation::UAV3DofModel::StateVec{0.0, 0.0, 0.0, 200.0, 0.0, 0.0}));
}

TEST_F(InterceptorConfigTest, ParametersAreReadOnly)
{
    rclcpp::NodeOptions options;
    rclcpp::Node node{"interceptor_config_test", options};
    (void)gnc_ros::readInterceptorConfig(node);
    EXPECT_FALSE(node.set_parameter({"interceptor.mass_kg", 10.0}).successful);
}

}  // namespace
