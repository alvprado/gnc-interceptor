#include "simulation/uav_3dof_model.hpp"

#include "math/constants.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

namespace simulation
{
namespace
{

using State = UAV3DofModel::StateVec;
using Control = UAV3DofModel::ControlVec;

constexpr double k_tol{1.0e-12};
constexpr double k_half_pi{std::numbers::pi / 2.0};

/// @brief Drag at a given speed for the supplied parameters.
[[nodiscard]] double drag_at(UAV3DofModelParams const& p, double v)
{
    return 0.5 * p.rho_kgpm3 * p.frontal_area_m2 * p.drag_coeff * v * v;
}

/// @brief State vector built from its named components.
[[nodiscard]] State make_state(double x, double y, double z, double v, double psi, double gamma)
{
    State s;
    s << x, y, z, v, psi, gamma;
    return s;
}

/// @brief Control vector built from its named components.
[[nodiscard]] Control make_control(double thrust, double n, double bank)
{
    Control u;
    u << thrust, n, bank;
    return u;
}

class UAV3DofModelTest : public ::testing::Test
{
protected:
    UAV3DofModelParams params_{};
    UAV3DofModelLimits limits_{};
    UAV3DofModel model_{params_, limits_};
};

TEST_F(UAV3DofModelTest, TranslationalKinematicsMatchVelocityVector)
{
    constexpr double v{80.0};
    constexpr double psi{0.7};
    constexpr double gamma{0.3};
    auto const dx =
        model_(make_state(10.0, 20.0, 30.0, v, psi, gamma), make_control(0.0, 0.0, 0.0));

    EXPECT_NEAR(dx[0], v * std::cos(gamma) * std::cos(psi), k_tol);
    EXPECT_NEAR(dx[1], v * std::cos(gamma) * std::sin(psi), k_tol);
    EXPECT_NEAR(dx[2], v * std::sin(gamma), k_tol);
}

TEST_F(UAV3DofModelTest, PositionDoesNotAffectDerivative)
{
    auto const u = make_control(50.0, 2.0, 0.4);
    auto const a = model_(make_state(0.0, 0.0, 0.0, 90.0, 0.2, 0.1), u);
    auto const b = model_(make_state(1e4, -5e3, 2e3, 90.0, 0.2, 0.1), u);

    EXPECT_TRUE(a.isApprox(b)) << "position must not enter f(x, u)";
}

TEST_F(UAV3DofModelTest, SpeedRateIsThrustMinusDragMinusGravity)
{
    constexpr double v{120.0};
    constexpr double gamma{0.25};
    constexpr double thrust{70.0};
    auto const dx =
        model_(make_state(0.0, 0.0, 0.0, v, 0.0, gamma), make_control(thrust, 1.0, 0.0));

    double const expected = (thrust - drag_at(params_, v)) / params_.mass_kg -
                            math::k_gravity_mps2 * std::sin(gamma);
    EXPECT_NEAR(dx[3], expected, k_tol);
}

TEST_F(UAV3DofModelTest, LevelFlightAtUnitLoadFactorHoldsAttitude)
{
    constexpr double v{100.0};
    auto const dx =
        model_(make_state(0.0, 0.0, 0.0, v, 0.0, 0.0), make_control(drag_at(params_, v), 1.0, 0.0));

    EXPECT_NEAR(dx[3], 0.0, k_tol) << "thrust matches drag";
    EXPECT_NEAR(dx[4], 0.0, k_tol) << "no bank, no turn";
    EXPECT_NEAR(dx[5], 0.0, k_tol) << "lift balances weight";
    EXPECT_NEAR(dx[0], v, k_tol);
}

TEST_F(UAV3DofModelTest, ClimbDeceleratesMoreThanDive)
{
    auto const u = make_control(0.0, 1.0, 0.0);
    auto const climbing = model_(make_state(0.0, 0.0, 0.0, 100.0, 0.0, 0.4), u);
    auto const diving = model_(make_state(0.0, 0.0, 0.0, 100.0, 0.0, -0.4), u);

    EXPECT_LT(climbing[3], 0.0) << "gravity and drag both oppose a climb";
    EXPECT_LT(climbing[3], diving[3]) << "gravity aids a dive";
}

TEST_F(UAV3DofModelTest, DerivativeStaysFiniteAtSingularities)
{
    struct Sample
    {
        char const* name;
        State x;
    };
    auto const u = make_control(50.0, 5.0, 0.5);
    Sample const samples[]{
        {"zero speed", make_state(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)},
        {"tiny speed", make_state(0.0, 0.0, 0.0, 1.0e-12, 0.0, 0.0)},
        {"vertical up", make_state(0.0, 0.0, 0.0, 100.0, 0.0, k_half_pi)},
        {"vertical down", make_state(0.0, 0.0, 0.0, 100.0, 0.0, -k_half_pi)},
        {"both singular", make_state(0.0, 0.0, 0.0, 0.0, 0.0, k_half_pi)},
    };

    for (auto const& s : samples)
    {
        EXPECT_TRUE(model_(s.x, u).allFinite()) << "non-finite derivative at: " << s.name;
    }
}

TEST_F(UAV3DofModelTest, HeadingRateStaysBoundedAtVertical)
{
    // A finiteness check cannot catch this: unguarded, 1 / cos(pi/2) is ~1e16,
    // which is enormous but still finite.
    constexpr double k_max_plausible_rate{1.0e3};
    auto const u = make_control(50.0, 5.0, 0.5);
    double const gammas[]{k_half_pi, -k_half_pi};

    for (double const gamma : gammas)
    {
        auto const dx = model_(make_state(0.0, 0.0, 0.0, 100.0, 0.0, gamma), u);
        EXPECT_LT(std::abs(dx[4]), k_max_plausible_rate) << "unbounded rate at gamma = " << gamma;
    }
}

TEST_F(UAV3DofModelTest, HeadingRateReversesPastVertical)
{
    auto const u = make_control(50.0, 5.0, 0.5);
    double const below = model_(make_state(0.0, 0.0, 0.0, 100.0, 0.0, k_half_pi - 0.2), u)[4];
    double const above = model_(make_state(0.0, 0.0, 0.0, 100.0, 0.0, k_half_pi + 0.2), u)[4];

    EXPECT_LT(below * above, 0.0) << "cos(gamma) changes sign either side of vertical";
    EXPECT_NEAR(below, -above, k_tol) << "and the magnitude is symmetric";
}

TEST_F(UAV3DofModelTest, DerivativeIsUnaffectedByLimits)
{
    UAV3DofModelLimits tight{};
    tight.max_load_factor = 1.0;
    UAV3DofModel const restricted{params_, tight};

    auto const x = make_state(0.0, 0.0, 0.0, 100.0, 0.1, 0.2);
    auto const u = make_control(50.0, 9.0, 0.5);
    EXPECT_TRUE(model_(x, u).isApprox(restricted(x, u)))
        << "f(x, u) must not clamp; only step() applies the envelope";
}

TEST_F(UAV3DofModelTest, ClampControlLeavesFeasibleCommandsUntouched)
{
    auto const u = make_control(60.0, 2.0, 0.3);
    EXPECT_TRUE(model_.clampControl(u).isApprox(u));
}

TEST_F(UAV3DofModelTest, ClampControlLimitsEachChannel)
{
    auto const high = model_.clampControl(make_control(1.0e4, 50.0, 0.4));
    EXPECT_DOUBLE_EQ(high[0], limits_.max_thrust_n);
    EXPECT_DOUBLE_EQ(high[1], limits_.max_load_factor);
    EXPECT_DOUBLE_EQ(high[2], 0.4) << "bank is within the default envelope";

    auto const low = model_.clampControl(make_control(-500.0, -50.0, 0.0));
    EXPECT_DOUBLE_EQ(low[0], limits_.min_thrust_n);
    EXPECT_DOUBLE_EQ(low[1], limits_.min_load_factor);
}

TEST_F(UAV3DofModelTest, ClampStateLeavesFeasibleStatesUntouched)
{
    auto const x = make_state(100.0, 200.0, 300.0, 120.0, 2.0, 0.4);
    EXPECT_TRUE(model_.clampState(x).isApprox(x));
}

TEST_F(UAV3DofModelTest, ClampStateLimitsSpeedAndFlightPathAngle)
{
    auto const slow = model_.clampState(make_state(0.0, 0.0, 0.0, -50.0, 0.0, 0.0));
    EXPECT_DOUBLE_EQ(slow[3], limits_.min_speed_mps);

    auto const fast = model_.clampState(make_state(0.0, 0.0, 0.0, 1.0e4, 0.0, 0.0));
    EXPECT_DOUBLE_EQ(fast[3], limits_.max_speed_mps);

    auto const steep = model_.clampState(make_state(0.0, 0.0, 0.0, 100.0, 0.0, k_half_pi));
    EXPECT_DOUBLE_EQ(steep[5], limits_.max_flight_path_angle_rad);

    auto const dive = model_.clampState(make_state(0.0, 0.0, 0.0, 100.0, 0.0, -k_half_pi));
    EXPECT_DOUBLE_EQ(dive[5], -limits_.max_flight_path_angle_rad);
}

TEST_F(UAV3DofModelTest, ClampStateLeavesPositionAndHeadingAlone)
{
    auto const x = make_state(1.0e4, -2.0e4, 3.0e3, 1.0e4, 40.0, 0.0);
    auto const clamped = model_.clampState(x);

    EXPECT_DOUBLE_EQ(clamped[0], x[0]);
    EXPECT_DOUBLE_EQ(clamped[1], x[1]);
    EXPECT_DOUBLE_EQ(clamped[2], x[2]);
    EXPECT_DOUBLE_EQ(clamped[4], x[4]) << "heading is not wrapped by the model";
}

}  // namespace
}  // namespace simulation
