#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "simulation/simulation_node.hpp"
#include "simulation/parameters/target_config.hpp"

namespace
{

class TargetConfigTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    static gnc_ros::TargetConfig read(std::vector<rclcpp::Parameter> const& parameters)
    {
        rclcpp::NodeOptions options;
        options.parameter_overrides(parameters);
        rclcpp::Node node{"target_config_test", options};
        return gnc_ros::readTargetConfig(node);
    }

    template <target::TargetTrajectory T>
    static void expectSameTrajectory(gnc_ros::TargetConfig const& config, T const& expected)
    {
        auto const actual = gnc_ros::makeTargetTrajectory(config);
        for (double const time_s : {0.0, 1.5, 10.0})
        {
            auto const state = actual.evaluateTargetStateAt(time_s);
            auto const reference = expected.evaluateTargetStateAt(time_s);
            EXPECT_TRUE(state.position_m.isApprox(reference.position_m));
            EXPECT_TRUE(state.velocity_mps.isApprox(reference.velocity_mps));
            EXPECT_TRUE(state.acceleration_mps2.isApprox(reference.acceleration_mps2));
        }
    }
};

TEST_F(TargetConfigTest, DefaultsMatchStandaloneFigureEight)
{
    auto const config = read({});
    EXPECT_EQ(config.type, gnc_ros::TargetType::FigureEight);
    expectSameTrajectory(
        config,
        target::FigureEight{
            {3000.0, 500.0, 1500.0}, 1000.0, 500.0, 0.1, {1.0, 0.0, 1.0}, {0.0, -500.0, 0.0}});
}

TEST_F(TargetConfigTest, ReadsConstantVelocity)
{
    auto const config = read({
        {"target.type", "constant_velocity"},
        {"target.position_m", std::vector<double>{1.0, 2.0, 3.0}},
        {"target.velocity_mps", std::vector<double>{4.0, -5.0, 6.0}},
    });
    EXPECT_EQ(config.type, gnc_ros::TargetType::ConstantVelocity);
    auto const trajectory = gnc_ros::makeTargetTrajectory(config);
    auto const state = trajectory.evaluateTargetStateAt(2.0);
    EXPECT_TRUE(state.position_m.isApprox(Eigen::Vector3d{9.0, -8.0, 15.0}));
    EXPECT_TRUE(state.velocity_mps.isApprox(Eigen::Vector3d{4.0, -5.0, 6.0}));
    EXPECT_TRUE(state.acceleration_mps2.isZero());
}

TEST_F(TargetConfigTest, ReadsCircle)
{
    auto const config = read({
        {"target.type", "circle"},
        {"target.center_m", std::vector<double>{10.0, 20.0, 30.0}},
        {"target.normal", std::vector<double>{0.0, 0.0, 1.0}},
        {"target.reference_direction", std::vector<double>{1.0, 0.0, 0.0}},
        {"target.speed_mps", 40.0},
        {"target.load_factor", -2.0},
    });
    EXPECT_EQ(config.type, gnc_ros::TargetType::Circle);
    expectSameTrajectory(
        config, target::Circle{{10.0, 20.0, 30.0}, 40.0, -2.0, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}});
}

TEST_F(TargetConfigTest, ReadsFigureEight)
{
    auto const config = read({
        {"target.type", "figure_eight"},
        {"target.center_m", std::vector<double>{-10.0, 20.0, 30.0}},
        {"target.normal", std::vector<double>{0.0, 1.0, 0.0}},
        {"target.reference_direction", std::vector<double>{1.0, 0.0, 1.0}},
        {"target.length_m", 200.0},
        {"target.width_m", 80.0},
        {"target.angular_rate_rps", -0.2},
    });
    expectSameTrajectory(
        config, target::FigureEight{
                    {-10.0, 20.0, 30.0}, 200.0, 80.0, -0.2, {0.0, 1.0, 0.0}, {1.0, 0.0, 1.0}});
}

TEST_F(TargetConfigTest, ReadsHelix)
{
    auto const config = read({
        {"target.type", "helix"},
        {"target.center_m", std::vector<double>{10.0, 20.0, 30.0}},
        {"target.normal", std::vector<double>{0.0, 0.0, 1.0}},
        {"target.reference_direction", std::vector<double>{1.0, 0.0, 0.0}},
        {"target.speed_mps", 60.0},
        {"target.load_factor", 1.5},
        {"target.climb_angle_rad", -0.3},
    });
    EXPECT_EQ(config.type, gnc_ros::TargetType::Helix);
    expectSameTrajectory(
        config,
        target::Helix{{10.0, 20.0, 30.0}, 60.0, 1.5, -0.3, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}});
}

TEST_F(TargetConfigTest, ValidConfigCreatesSimulationInterfaces)
{
    gnc_ros::SimulationNode node;
    EXPECT_EQ(node.count_publishers("/clock"), 1U);
    EXPECT_EQ(node.count_publishers("target/ground_truth"), 1U);
    EXPECT_EQ(node.count_subscribers("guidance/command"), 1U);
}

TEST_F(TargetConfigTest, ParametersAreReadOnlyAndSpecificToSelectedType)
{
    rclcpp::NodeOptions options;
    options.parameter_overrides({{"target.type", "constant_velocity"}});
    rclcpp::Node node{"target_config_test", options};
    (void)gnc_ros::readTargetConfig(node);
    EXPECT_FALSE(node.has_parameter("target.length_m"));
    EXPECT_FALSE(node.set_parameter({"target.type", "figure_eight"}).successful);
    EXPECT_FALSE(
        node.set_parameter({"target.velocity_mps", std::vector<double>{1.0, 2.0, 3.0}}).successful);
}

}  // namespace
