#include <cstdio>
#include <numbers>
#include <string_view>
#include <vector>

#include "csv_logger.hpp"
#include "math/constants.hpp"
#include "math/integrators.hpp"
#include "simulation/simulator.hpp"
#include "simulation/uav_3dof_model.hpp"
#include "target/maneuvers.hpp"

namespace
{
/// @brief Compute thrust and load factor to remain in a constant speed and flight path angle via
/// feedforward control
[[nodiscard]] Eigen::Vector<double, 3> steadyFlightControl(
    Eigen::Vector<double, 6> const& state, simulation::UAV3DofModelParams const& params) noexcept
{
    auto const& flight_path_angle = state[5];
    auto const& velocity = state[3];

    double const thrust =
        math::gravity_mps2 * std::sin(flight_path_angle) * params.mass_kg +
        0.5 * params.rho_kgpm3 * params.drag_coeff * params.frontal_area_m2 * velocity * velocity;
    double const load_factor = std::cos(flight_path_angle);

    return Eigen::Vector3d{thrust, load_factor, 0.0};
}

}  // namespace

int main()
{
    using simulation::UAV3DofModel;
    using simulation::UAVSimulator;

    // Interceptor: default physical parameters and envelope, RK4 integration.
    UAV3DofModel const model{simulation::UAV3DofModelParams{}, simulation::UAV3DofModelLimits{}};
    UAVSimulator<UAV3DofModel, math::RK4Step> const sim{model, math::RK4Step{}};

    // Start climbing at 20 deg, away from the |gamma| = 90 deg singularity.
    UAV3DofModel::StateVec initial_state;
    initial_state << 0.0, 0.0, 0.0, 1.0, 0.0, 20.0 * std::numbers::pi / 180.0;
    math::CartesianState state = model.toCartesianState(initial_state);

    // Initial thrust
    constexpr double initial_thrust{100.0};

    // Target: a horizontal figure-eight ahead of the interceptor.
    target::FigureEight const target_traj{
        Eigen::Vector3d{3000.0, 0.0, 1500.0}, 1000.0, 500.0, 0.15, Eigen::Vector3d{1.0, 0.0, 1.0},
        Eigen::Vector3d{1.0, 0.0, 0.0}};

    constexpr double dt{0.1};
    constexpr double duration_s{50.0};
    constexpr int steps{static_cast<int>(duration_s / dt)};

    std::vector<TrajectorySample> samples;
    samples.reserve(static_cast<std::size_t>(steps) + 1);

    std::printf("%6s  %10s %10s %10s  %8s  %10s\n", "t[s]", "int_x", "int_y", "int_z", "int_v",
                "int_gam[deg]");

    double t = 0.0;
    for (int i = 0; i <= steps; ++i)
    {
        UAV3DofModel::StateVec const model_state = model.fromCartesianState(state);

        auto const target_state = target_traj.evaluateTargetStateAt(t);
        samples.push_back(TrajectorySample{t, model_state, target_state});

        if (i % 10 == 0)
        {
            std::printf("%6.1f  %10.2f %10.2f %10.2f  %8.2f  %10.2f\n", t, model_state[0],
                        model_state[1], model_state[2], model_state[3],
                        model_state[5] * 180.0 / std::numbers::pi);
        }

        if (i == steps)
        {
            break;
        }

        auto const& control =
            (t < 5.0) ? UAV3DofModel::ControlVec{initial_thrust, 1.0, 0.0}
                      : steadyFlightControl(model_state, simulation::UAV3DofModelParams{});
        state = sim.step(state, control, dt);
        t += dt;
    }

    constexpr std::string_view csv_path{"standalone/outputs/standalone_trajectory.csv"};
    writeTrajectoryCsv(csv_path, samples);
    std::printf("\nwrote %s\n", csv_path.data());
    return 0;
}
