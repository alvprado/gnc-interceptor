#include "guidance/transverse_control_allocation.hpp"

#include "math/cartesian_state.hpp"
#include "math/constants.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

namespace guidance
{
namespace
{

constexpr double k_tol{1.0e-9};

/// @brief Cartesian state built from a velocity, at the origin.
[[nodiscard]] math::CartesianState make_cartesian(Eigen::Vector3d const& velocity_mps)
{
    math::CartesianState state;
    state.velocity_mps = velocity_mps;
    return state;
}

class TransverseControlAllocationTest : public ::testing::Test
{
protected:
    TransverseControlAllocationConfig config_{};
    TransverseControlAllocation allocation_{config_};
};

TEST_F(TransverseControlAllocationTest, ZeroCommandInLevelFlightYieldsUnitLoadFactorAndZeroBank)
{
    auto const interceptor = make_cartesian(Eigen::Vector3d{100.0, 0.0, 0.0});

    auto const out = allocation_.step(interceptor, Eigen::Vector3d::Zero());

    EXPECT_NEAR(out.load_factor, 1.0, k_tol);
    EXPECT_NEAR(out.bank_angle_rad, 0.0, k_tol);
}

TEST_F(TransverseControlAllocationTest, ZeroCommandWhileClimbingYieldsCosGammaLoadFactor)
{
    constexpr double gamma{30.0 * std::numbers::pi / 180.0};
    auto const interceptor =
        make_cartesian(Eigen::Vector3d{100.0 * std::cos(gamma), 0.0, 100.0 * std::sin(gamma)});

    auto const out = allocation_.step(interceptor, Eigen::Vector3d::Zero());

    EXPECT_NEAR(out.load_factor, std::cos(gamma), k_tol);
    EXPECT_NEAR(out.bank_angle_rad, 0.0, k_tol);
}

TEST_F(TransverseControlAllocationTest, PullUpCommandIncreasesLoadFactorWithZeroBank)
{
    auto const interceptor = make_cartesian(Eigen::Vector3d{100.0, 0.0, 0.0});
    // Level flight: e_gamma = (0, 0, 1), so this is a pure "pull up" command.
    Eigen::Vector3d const a_c{0.0, 0.0, 10.0};

    auto const out = allocation_.step(interceptor, a_c);

    EXPECT_NEAR(out.load_factor, (10.0 + math::gravity_mps2) / math::gravity_mps2, k_tol);
    EXPECT_NEAR(out.bank_angle_rad, 0.0, k_tol);
}

TEST_F(TransverseControlAllocationTest, GravityCancelledSideCommandYieldsNinetyDegreeBank)
{
    auto const interceptor = make_cartesian(Eigen::Vector3d{100.0, 0.0, 0.0});
    // Cancel the level-flight trim term (g) and add a pure side (e_psi) command.
    Eigen::Vector3d const a_c{0.0, 10.0, -math::gravity_mps2};

    auto const out = allocation_.step(interceptor, a_c);

    EXPECT_NEAR(out.load_factor, 10.0 / math::gravity_mps2, k_tol);
    EXPECT_NEAR(out.bank_angle_rad, std::numbers::pi / 2.0, k_tol);
}

TEST_F(TransverseControlAllocationTest, LoadFactorIsAlwaysNonNegative)
{
    auto const interceptor = make_cartesian(Eigen::Vector3d{100.0, 0.0, 0.0});
    // A strong "pushover" command: negative e_gamma direction.
    Eigen::Vector3d const a_c{0.0, 0.0, -50.0};

    auto const out = allocation_.step(interceptor, a_c);

    EXPECT_GE(out.load_factor, 0.0);
    EXPECT_NEAR(out.bank_angle_rad, std::numbers::pi, k_tol)
        << "a pushover is represented as inverted bank, not negative load factor";
}

TEST_F(TransverseControlAllocationTest, LoadFactorIsClampedToConfiguredMaximum)
{
    auto const interceptor = make_cartesian(Eigen::Vector3d{100.0, 0.0, 0.0});
    Eigen::Vector3d const a_c{0.0, 0.0, 1.0e4};

    auto const out = allocation_.step(interceptor, a_c);

    EXPECT_DOUBLE_EQ(out.load_factor, config_.max_load_factor);
}

TEST_F(TransverseControlAllocationTest, BankAngleIsClampedToConfiguredMaximum)
{
    TransverseControlAllocationConfig tight{};
    tight.max_bank_angle_rad = std::numbers::pi / 4.0;
    TransverseControlAllocation const tight_allocation{tight};

    auto const interceptor = make_cartesian(Eigen::Vector3d{100.0, 0.0, 0.0});
    Eigen::Vector3d const a_c{0.0, 10.0, -math::gravity_mps2};  // wants 90 degrees of bank

    auto const out = tight_allocation.step(interceptor, a_c);

    EXPECT_DOUBLE_EQ(out.bank_angle_rad, tight.max_bank_angle_rad);
}

TEST_F(TransverseControlAllocationTest, StaysFiniteWhenInterceptorVelocityIsZero)
{
    auto const interceptor = make_cartesian(Eigen::Vector3d::Zero());
    auto const out = allocation_.step(interceptor, Eigen::Vector3d{1.0, 2.0, 3.0});

    EXPECT_TRUE(std::isfinite(out.load_factor));
    EXPECT_TRUE(std::isfinite(out.bank_angle_rad));
}

TEST_F(TransverseControlAllocationTest, StaysFiniteAtVerticalFlight)
{
    auto const interceptor = make_cartesian(Eigen::Vector3d{0.0, 0.0, 100.0});
    auto const out = allocation_.step(interceptor, Eigen::Vector3d{1.0, 2.0, 3.0});

    EXPECT_TRUE(std::isfinite(out.load_factor));
    EXPECT_TRUE(std::isfinite(out.bank_angle_rad));
}

}  // namespace
}  // namespace guidance
