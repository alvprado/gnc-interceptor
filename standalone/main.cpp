#include <algorithm>
#include <cstdio>
#include <numbers>
#include <string_view>
#include <vector>

#include "csv_logger.hpp"
#include "guidance/pn_controller.hpp"
#include "guidance/predictive_guidance_controller.hpp"
#include "math/cartesian_state.hpp"
#include "math/constants.hpp"
#include "math/integrators.hpp"
#include "simulation/simulator.hpp"
#include "simulation/uav_3dof_model.hpp"
#include "target/maneuvers.hpp"
namespace
{

[[nodiscard]] bool interceptionOccured(math::CartesianState const& target,
                                       math::CartesianState const& interceptor)
{
    constexpr double interception_distance{1.0};
    return (target.position_m - interceptor.position_m).norm() < interception_distance;
}

/// @brief Interceptor state at the origin, moving at 1 m/s toward the target.
[[nodiscard]] math::CartesianState initializeInterceptorState(
    math::CartesianState const& target_state) noexcept
{
    constexpr double initial_speed_mps{1.0};
    constexpr double min_range_m{1.0e-6};

    double const range = std::max(target_state.position_m.norm(), min_range_m);
    Eigen::Vector3d const direction = target_state.position_m / range;

    math::CartesianState interceptor_state;
    interceptor_state.position_m = Eigen::Vector3d::Zero();
    interceptor_state.velocity_mps = initial_speed_mps * direction;
    return interceptor_state;
}

}  // namespace

int main()
{
    using guidance::PNController;
    using guidance::PredictiveGuidanceController;
    using simulation::UAV3DofModel;
    using simulation::UAVSimulator;

    // Interceptor: default physical parameters and envelope, RK4 integration.
    UAV3DofModel const model{simulation::UAV3DofModelParams{}, simulation::UAV3DofModelLimits{}};
    UAVSimulator<UAV3DofModel, math::RK4Step> const sim{model, math::RK4Step{}};

    // Flag to switch controllers
    constexpr bool use_predictive_guidance{true};

    /// Standard guidance controller
    PNController const pn_controller{guidance::PNControllerConfig{}};

    // Predictive guidance controller
    auto ilqr_config = guidance::PredictiveGuidanceControllerConfig{};
    ilqr_config.solver_config.max_iterations = 50;
    PredictiveGuidanceController predictive_controller{ilqr_config};

    // Target: figure-eight
    target::FigureEight const target_traj{
        Eigen::Vector3d{3000.0, 500.0, 1500.0},  // center_m
        1000.0,                                  // length_m (tip to tip along the long axis)
        500.0,                                   // width_m (tip to tip across)
        0.1,                                     // angular_rate_rps
        Eigen::Vector3d{1.0, 0.0, 1.0},          // normal (flat, horizontal figure eight)
        Eigen::Vector3d{0.0, -500.0, 0.0}};      // reference_direction (fixes the long axis)

    // Interceptor: at the origin, launched pointing at the target's initial
    // position.
    math::CartesianState state = initializeInterceptorState(target_traj.evaluateTargetStateAt(0.0));

    constexpr double dt{0.01};
    constexpr double duration_s{100.0};
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
        Eigen::Vector3d const control = use_predictive_guidance
                                            ? predictive_controller.step(target_state, state, dt)
                                            : pn_controller.step(target_state, state, dt);
        samples.push_back(TrajectorySample{t, model_state, target_state, control});

        if (i % 100 == 0)
        {
            std::printf("%6.1f  %10.2f %10.2f %10.2f  %8.2f  %10.2f\n", t, model_state[0],
                        model_state[1], model_state[2], model_state[3],
                        model_state[5] * 180.0 / std::numbers::pi);
        }

        if (i == steps)
        {
            break;
        }

        state = sim.step(state, control, dt);
        t += dt;

        if (interceptionOccured(target_state, state))
        {
            std::printf("\ninterception occurred!\n");
            break;
        }
    }

    constexpr std::string_view csv_path{"standalone/outputs/standalone_trajectory.csv"};
    writeTrajectoryCsv(csv_path, samples);
    std::printf("\nwrote %s\n", csv_path.data());
    return 0;
}
