#include "guidance/pn_controller.hpp"

#include "guidance/pn_control_law.hpp"
#include "guidance/thrust_control_law.hpp"
#include "guidance/transverse_control_allocation.hpp"
#include "math/cartesian_state.hpp"

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

    EXPECT_DOUBLE_EQ(control[0], config_.thrust_control_config.max_thrust_n);
    EXPECT_DOUBLE_EQ(control[1], 1.0);
    EXPECT_DOUBLE_EQ(control[2], 0.0);
}

TEST_F(PNControllerTest, AboveSwitchSpeedMatchesManualComposition)
{
    auto const interceptor = make_cartesian(
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d{config_.thrust_control_config.switch_speed_mps + 20.0, 0.0, 0.0});
    auto const target = make_cartesian(Eigen::Vector3d{1000.0, 100.0, 0.0},
                                       Eigen::Vector3d{-100.0, 0.0, 0.0});

    ProportionalNavigationControlLaw const pn_law{config_.navigation_gain};
    ThrustControlLaw const thrust_law{config_.thrust_control_config};
    TransverseControlAllocation const allocation{config_.transverse_control_allocation_config};

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
        Eigen::Vector3d{config_.thrust_control_config.switch_speed_mps + 20.0, 0.0, 0.0});
    auto const target = make_cartesian(Eigen::Vector3d{1000.0, 100.0, 0.0},
                                       Eigen::Vector3d{-100.0, 0.0, 0.0});

    auto const control = controller_.step(target, interceptor, 0.1);

    EXPECT_NE(control[1], 1.0) << "PN should command a load factor away from the boost default";
}

}  // namespace
}  // namespace guidance
