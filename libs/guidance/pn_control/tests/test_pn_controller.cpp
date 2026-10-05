#include "guidance/pn_controller.hpp"

#include "guidance/pn_control_law.hpp"
#include "guidance/thrust_control_law.hpp"
#include "guidance/transverse_control_allocation.hpp"
#include "math/state_types.hpp"

#include <gtest/gtest.h>

namespace guidance
{
namespace
{

constexpr double k_tol{1.0e-9};

/// @brief Cartesian state built from position and velocity.
[[nodiscard]] math::CartesianState make_cartesian(Eigen::Vector3d const& position_m,
                                                   Eigen::Vector3d const& velocity_mps)
{
    math::CartesianState state;
    state.position_m = position_m;
    state.velocity_mps = velocity_mps;
    return state;
}

class PNControllerTest : public ::testing::Test
{
protected:
    PNControllerConfig config_{};
    PNController controller_{config_};
};

TEST_F(PNControllerTest, BelowSwitchSpeedCommandsWingsLevelBoost)
{
    auto const interceptor =
        make_cartesian(Eigen::Vector3d::Zero(), Eigen::Vector3d{1.0, 0.0, 0.0});
    auto const target = make_cartesian(Eigen::Vector3d{1000.0, 100.0, 0.0},
                                       Eigen::Vector3d{-100.0, 0.0, 0.0});

    auto const control = controller_.step(target, interceptor, 0.1);

    EXPECT_DOUBLE_EQ(control[0], config_.boost_phase_thrust_n);
    EXPECT_DOUBLE_EQ(control[1], 1.0);
    EXPECT_DOUBLE_EQ(control[2], 0.0);
}

TEST_F(PNControllerTest, AboveSwitchSpeedMatchesManualComposition)
{
    auto const interceptor = make_cartesian(
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d{config_.boost_phase_switch_speed_mps + 20.0, 0.0, 0.0});
    auto const target = make_cartesian(Eigen::Vector3d{1000.0, 100.0, 0.0},
                                       Eigen::Vector3d{-100.0, 0.0, 0.0});

    ProportionalNavigationControlLaw const pn_law{config_.navigation_gain};
    ThrustControlLaw const thrust_law{ThrustControlConfig{.vehicle = config_.vehicle}};
    TransverseControlAllocation const allocation{
        TransverseControlAllocationConfig{.min_load_factor = config_.min_load_factor,
                                          .max_load_factor = config_.max_load_factor,
                                          .max_bank_angle_rad = config_.max_bank_angle_rad}};

    auto const a_c = pn_law.step(target, interceptor);
    auto const allocation_out = allocation.step(interceptor, a_c);
    double const expected_thrust = thrust_law.step(interceptor);

    auto const control = controller_.step(target, interceptor, 0.1);

    EXPECT_NEAR(control[0], expected_thrust, k_tol);
    EXPECT_NEAR(control[1], allocation_out.load_factor, k_tol);
    EXPECT_NEAR(control[2], allocation_out.bank_angle_rad, k_tol);
}

TEST_F(PNControllerTest, AboveSwitchSpeedEngagesGuidanceForNonCollinearGeometry)
{
    auto const interceptor = make_cartesian(
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d{config_.boost_phase_switch_speed_mps + 20.0, 0.0, 0.0});
    auto const target = make_cartesian(Eigen::Vector3d{1000.0, 100.0, 0.0},
                                       Eigen::Vector3d{-100.0, 0.0, 0.0});

    auto const control = controller_.step(target, interceptor, 0.1);

    EXPECT_NE(control[1], 1.0) << "PN should command a load factor away from the boost default";
}

}  // namespace
}  // namespace guidance

TEST(PNControllerBoostTest, HoldsClimbAngleDuringBoostAndRemainsFiniteAtRest)
{
    guidance::PNControllerConfig const config;
    guidance::PNController controller{config};
    math::CartesianState interceptor;
    math::CartesianState target;
    target.position_m = {100.0, 0.0, 100.0};
    EXPECT_TRUE(controller.step(target, interceptor, 0.1).allFinite());
    interceptor.velocity_mps = {3.0, 0.0, 4.0};
    auto const control = controller.step(target, interceptor, 0.1);
    EXPECT_DOUBLE_EQ(control[0], config.boost_phase_thrust_n);
    EXPECT_NEAR(control[1], 0.6, 1.0e-12);
    EXPECT_DOUBLE_EQ(control[2], 0.0);
}
