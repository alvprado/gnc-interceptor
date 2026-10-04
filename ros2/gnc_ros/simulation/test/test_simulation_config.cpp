#include <gtest/gtest.h>

#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "simulation/parameters/simulation_config.hpp"
#include "simulation/simulation_node.hpp"

namespace
{

class SimulationConfigTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    static gnc_ros::SimConfig read(std::vector<rclcpp::Parameter> const& parameters)
    {
        rclcpp::NodeOptions options;
        options.parameter_overrides(parameters);
        rclcpp::Node node{"simulation_config_test", options};
        return gnc_ros::readSimulationConfig(node);
    }

    static double stepDragDecay(gnc_ros::SimConfig const& config)
    {
        using Model = simulation::UAV3DofModel;
        Model const model{{.mass_kg = 1.0, .rho_kgpm3 = 2.0,
                           .frontal_area_m2 = 1.0, .drag_coeff = 1.0}};
        simulation::UAVSimulator<Model, gnc_ros::SimConfig::Integrator> const simulator{
            model, config.integrator};
        math::VehicleState state;
        state.cartesian.velocity_mps = {2.0, 0.0, 0.0};
        return simulator.step(state, Model::ControlVec{0.0, 1.0, 0.0}, config.dt_s)
            .cartesian.velocity_mps.norm();
    }
};

TEST_F(SimulationConfigTest, DefaultsUseRk4At100Hz)
{
    auto const config = read({});
    EXPECT_DOUBLE_EQ(config.dt_s, 0.01);
    EXPECT_NEAR(stepDragDecay(config), 1.9607843139716112, 1.0e-14);
}

TEST_F(SimulationConfigTest, SelectedMethodsProduceTheirExpectedNumericalSteps)
{
    struct Case
    {
        char const* name;
        double expected;
    };
    // Level, unpowered flight with v' = -v^2, initial v = 2 and dt = 0.1.
    for (auto const& test : {Case{"euler", 1.6}, Case{"heun", 1.672},
                             Case{"rk4", 1.6666780712460774}})
    {
        SCOPED_TRACE(test.name);
        auto const config = read({{"simulation.dt_s", 0.1},
                                  {"simulation.integration_method", test.name}});
        EXPECT_DOUBLE_EQ(config.dt_s, 0.1);
        EXPECT_NEAR(stepDragDecay(config), test.expected, 1.0e-14);

        rclcpp::NodeOptions options;
        options.parameter_overrides({{"simulation.dt_s", 0.1},
                                     {"simulation.integration_method", test.name}});
        EXPECT_NO_THROW((gnc_ros::SimulationNode{options}));
    }
}

TEST_F(SimulationConfigTest, InvalidTimestepsFallBackToDefault)
{
    for (double const dt : {0.0, -0.01, 1.0e-12, std::numeric_limits<double>::max(),
                            std::numeric_limits<double>::infinity(),
                            std::numeric_limits<double>::quiet_NaN()})
    {
        auto const config = read({{"simulation.dt_s", dt},
                                  {"simulation.integration_method", "euler"}});
        EXPECT_DOUBLE_EQ(config.dt_s, 0.01);
        EXPECT_NEAR(stepDragDecay(config), 1.96, 1.0e-14);

        rclcpp::NodeOptions options;
        options.parameter_overrides({{"simulation.dt_s", dt}});
        EXPECT_NO_THROW((gnc_ros::SimulationNode{options}));
    }
}

TEST_F(SimulationConfigTest, UnknownMethodFallsBackToRk4)
{
    auto const config = read({{"simulation.dt_s", 0.1},
                              {"simulation.integration_method", "unknown"}});
    EXPECT_DOUBLE_EQ(config.dt_s, 0.1);
    EXPECT_NEAR(stepDragDecay(config), 1.6666780712460774, 1.0e-14);
}

TEST_F(SimulationConfigTest, ParametersAreReadOnly)
{
    rclcpp::Node node{"simulation_config_test"};
    (void)gnc_ros::readSimulationConfig(node);
    EXPECT_FALSE(node.set_parameter({"simulation.dt_s", 0.02}).successful);
    EXPECT_FALSE(node.set_parameter({"simulation.integration_method", "euler"}).successful);
}

}  // namespace
