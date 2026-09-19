#include "simulation/simulator.hpp"

#include "math/integrators.hpp"
#include "simulation/uav_3dof_model.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

namespace simulation
{
namespace
{

using State = UAV3DofModel::StateVec;
using Control = UAV3DofModel::ControlVec;
using RK4Sim = UAVSimulator<UAV3DofModel, math::RK4Step>;

static_assert(SimulatableModel<UAV3DofModel>,
              "the 3-DOF model must satisfy the simulator contract");

constexpr double k_half_pi{std::numbers::pi / 2.0};

[[nodiscard]] State make_state(double x, double y, double z, double v, double psi, double gamma)
{
    State s;
    s << x, y, z, v, psi, gamma;
    return s;
}

[[nodiscard]] Control make_control(double thrust, double n, double bank)
{
    Control u;
    u << thrust, n, bank;
    return u;
}

/// @brief Advance a state for a fixed number of steps.
template <typename Integrator_T>
[[nodiscard]] State propagate(UAV3DofModel const& model, Integrator_T const& integrator, State x,
                              Control const& u, double dt, int steps)
{
    UAVSimulator<UAV3DofModel, Integrator_T> const sim{model, integrator};
    for (int i = 0; i < steps; ++i)
    {
        x = sim.step(x, u, dt);
    }
    return x;
}

class SimulatorTest : public ::testing::Test
{
protected:
    /// @brief Thrust that exactly balances drag at the given speed.
    [[nodiscard]] double trim_thrust(double v) const
    {
        return 0.5 * params_.rho_kgpm3 * params_.frontal_area_m2 * params_.drag_coeff * v * v;
    }

    UAV3DofModelParams params_{};
    UAV3DofModelLimits limits_{};
    UAV3DofModel model_{params_, limits_};
};

TEST_F(SimulatorTest, ZeroTimestepLeavesTheStateUnchanged)
{
    RK4Sim const sim{model_, math::RK4Step{}};
    auto const x = make_state(1.0, 2.0, 3.0, 100.0, 0.3, 0.2);

    EXPECT_TRUE(sim.step(x, make_control(50.0, 2.0, 0.4), 0.0).isApprox(x));
}

TEST_F(SimulatorTest, SteadyLevelFlightMatchesTheClosedForm)
{
    constexpr double v{100.0};
    constexpr double dt{0.01};
    constexpr int steps{100};

    auto const xf = propagate(model_, math::RK4Step{}, make_state(0.0, 0.0, 0.0, v, 0.0, 0.0),
                              make_control(trim_thrust(v), 1.0, 0.0), dt, steps);

    EXPECT_NEAR(xf[0], v * dt * steps, 1.0e-9) << "range is exactly v * t";
    EXPECT_NEAR(xf[1], 0.0, 1.0e-12);
    EXPECT_NEAR(xf[2], 0.0, 1.0e-12);
    EXPECT_NEAR(xf[3], v, 1.0e-9) << "trim thrust holds the speed";
    EXPECT_NEAR(xf[4], 0.0, 1.0e-12);
    EXPECT_NEAR(xf[5], 0.0, 1.0e-12);
}

TEST_F(SimulatorTest, IntegratorsAchieveTheirOrderOfAccuracy)
{
    auto const x0 = make_state(0.0, 0.0, 0.0, 120.0, 0.3, 0.1);
    auto const u = make_control(50.0, 3.0, 0.5);
    constexpr double horizon_s{2.0};

    // Step count is the primary quantity: deriving it as horizon / dt truncates
    // and leaves the reference a step short, which swamps the higher-order errors.
    constexpr int reference_steps{200000};
    auto const reference =
        propagate(model_, math::RK4Step{}, x0, u, horizon_s / reference_steps, reference_steps);

    auto error_for = [&](auto integrator, int steps) {
        auto const xf = propagate(model_, integrator, x0, u, horizon_s / steps, steps);
        return (xf - reference).norm();
    };

    // Halving dt must shrink the error by ~2^order. RK4 is checked loosely
    // because at 1e-11 it is already at the reference's own accuracy floor.
    EXPECT_NEAR(error_for(math::EulerStep{}, 100) / error_for(math::EulerStep{}, 200), 2.0, 0.2);
    EXPECT_NEAR(error_for(math::HeunStep{}, 100) / error_for(math::HeunStep{}, 200), 4.0, 0.4);
    EXPECT_GT(error_for(math::RK4Step{}, 100) / error_for(math::RK4Step{}, 200), 8.0);
}

TEST_F(SimulatorTest, RK4IsFarMoreAccurateThanEuler)
{
    auto const x0 = make_state(0.0, 0.0, 0.0, 120.0, 0.3, 0.1);
    auto const u = make_control(50.0, 3.0, 0.5);
    auto const reference = propagate(model_, math::RK4Step{}, x0, u, 1.0e-5, 200000);

    auto const euler = (propagate(model_, math::EulerStep{}, x0, u, 0.01, 200) - reference).norm();
    auto const rk4 = (propagate(model_, math::RK4Step{}, x0, u, 0.01, 200) - reference).norm();

    EXPECT_LT(rk4, euler);
}

TEST_F(SimulatorTest, StepAppliesTheControlLimits)
{
    UAV3DofModelLimits tight{};
    tight.max_load_factor = 1.0;
    UAV3DofModel const restricted{params_, tight};

    auto const x = make_state(0.0, 0.0, 0.0, 100.0, 0.0, 0.0);
    auto const excessive = make_control(50.0, 9.0, 0.0);
    auto const feasible = make_control(50.0, 1.0, 0.0);

    RK4Sim const sim{restricted, math::RK4Step{}};
    EXPECT_TRUE(sim.step(x, excessive, 0.01).isApprox(sim.step(x, feasible, 0.01)))
        << "a command above the limit must behave as the limit";
}

TEST_F(SimulatorTest, StepKeepsTheStateInsideTheEnvelope)
{
    // Vertical climb with no lift: gamma_dot vanishes and the speed decays.
    auto const u = make_control(0.0, 0.0, 0.0);
    RK4Sim const sim{model_, math::RK4Step{}};

    auto x = make_state(0.0, 0.0, 0.0, 50.0, 0.0, k_half_pi);
    for (int i = 0; i < 400; ++i)
    {
        x = sim.step(x, u, 0.02);
        ASSERT_TRUE(x.allFinite()) << "diverged at step " << i;
        EXPECT_GE(x[3], limits_.min_speed_mps);
        EXPECT_LE(x[3], limits_.max_speed_mps);
        EXPECT_LE(std::abs(x[5]), std::numbers::pi) << "gamma stays wrapped into (-pi, pi]";
    }
}

TEST_F(SimulatorTest, ClampingDoesNotPerturbFlightInsideTheEnvelope)
{
    UAV3DofModelLimits wide{};
    wide.max_thrust_n = 1.0e6;
    wide.max_load_factor = 1.0e3;
    wide.min_speed_mps = 1.0e-6;
    wide.max_speed_mps = 1.0e6;
    UAV3DofModel const unrestricted{params_, wide};

    auto const x0 = make_state(0.0, 0.0, 0.0, 120.0, 0.3, 0.1);
    auto const u = make_control(50.0, 3.0, 0.5);

    auto const limited = propagate(model_, math::RK4Step{}, x0, u, 0.01, 100);
    auto const unlimited = propagate(unrestricted, math::RK4Step{}, x0, u, 0.01, 100);

    EXPECT_TRUE(limited.isApprox(unlimited))
        << "an inactive envelope must cost nothing in accuracy";
}

}  // namespace
}  // namespace simulation
