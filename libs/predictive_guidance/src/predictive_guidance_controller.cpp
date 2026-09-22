#include "guidance/predictive_guidance_controller.hpp"

#include <algorithm>
#include <cmath>
#include <ilqr/dynamics/autodiff_policy.hpp>
#include <ilqr/ilqr.hpp>
#include <limits>
#include <numbers>
#include <span>
#include <utility>

#include "guidance/cost.hpp"
#include "guidance/dynamics_model.hpp"

namespace guidance
{
namespace
{

/// @brief Floor on interceptor speed (m/s) applied before normalizing.
inline constexpr double min_speed_mps{1.0e-6};

using Model = AugmentedDiscreteUAV3DofModel<ilqr::math::HeunStep, ilqr::AutoDiff>;
using Cost = ilqr::CompositeCostFunction<SoftminCost, ilqr::ControlPenaltyCost<Dims>>;
using Solver = ilqr::ILQRSolver<Model, Cost>;

}  // namespace

PredictiveGuidanceController::PredictiveGuidanceController(
    PredictiveGuidanceControllerConfig config)
    : config_(std::move(config)), target_predictions_(static_cast<std::size_t>(config_.horizon))
{
    // Sensible default warm start
    double const initial_flight_path_angle_rad = 20.0 * std::numbers::pi / 180.0;
    Dims::ControlVec const initial_control{
        trimThrust(config_.switch_speed_mps, initial_flight_path_angle_rad), 1.0, 0.0};
    previous_control_trajectory_.assign(static_cast<std::size_t>(config_.horizon), initial_control);
}

Eigen::Vector3d PredictiveGuidanceController::step(math::CartesianState const& target,
                                                   math::CartesianState const& interceptor, double)
{
    /// Compute internal model state out of cartesian state
    Dims::StateVec const x0 = toAugmentedState(interceptor);
    auto const& speed = x0[3];
    auto const& flight_path_angle_rad = x0[5];

    /// Boost phase on
    if (speed < config_.switch_speed_mps)
    {
        return Eigen::Vector3d{config_.max_thrust_n, 1.0, 0.0};
    }

    /// Predict target positions
    predictTargetPositions(target);

    /// Compute softmin config - min_q is the minimal scaled squared distance between the current
    /// interceptor trajectory and the predicted target trajectory
    SoftminConfig const softmin_config{config_.softmin_config.d_scale, computeMinQ(x0),
                                       config_.softmin_config.beta};

    /// Create an UAV 3DoF model with an augmented state for softmin computation
    Model model(UAV3DofModel{config_.vehicle},
                std::span<Eigen::Vector3d const>(target_predictions_), config_.dt, softmin_config,
                ilqr::math::HeunStep{}, ilqr::AutoDiff{});

    /// Construct the cost function - (soft)min interception distance + control effort
    SoftminCost softmin_cost(softmin_config, target_predictions_.back());
    ilqr::ControlPenaltyCost<Dims> control_cost(
        Dims::ControlMat(config_.control_effort_weight.asDiagonal()));
    Cost cost(std::move(softmin_cost), std::move(control_cost));

    /// Construct solver
    Solver solver(std::move(model), std::move(cost), config_.solver_config);

    /// Construct limits
    Dims::ControlVec const lower{0.0, config_.transverse_limits.min_load_factor,
                                 -config_.transverse_limits.max_bank_angle_rad};
    Dims::ControlVec const upper{trimThrust(speed, flight_path_angle_rad),
                                 config_.transverse_limits.max_load_factor,
                                 config_.transverse_limits.max_bank_angle_rad};

    /// Create a solve request - warm started from the previous solution
    auto const request = ilqr::SolveRequest<Dims>::warm_start(x0, previous_control_trajectory_)
                             .with_control_bounds(lower, upper);

    /// Solve
    auto const result = solver.solve(request);

    /// Update control and states
    previous_control_trajectory_.assign(result.trajectory.controls().begin(),
                                        result.trajectory.controls().end());
    previous_state_trajectory_.assign(result.trajectory.states().begin(),
                                      result.trajectory.states().end());

    /// Return first control input
    return result.trajectory.control(0);
}

double PredictiveGuidanceController::trimThrust(double speed_mps,
                                                double flight_path_angle_rad) const noexcept
{
    double const drag_force = 0.5 * config_.vehicle.rho_kgpm3 * config_.vehicle.frontal_area_m2 *
                              config_.vehicle.drag_coeff * speed_mps * speed_mps;
    return drag_force +
           math::gravity_mps2 * std::sin(flight_path_angle_rad) * config_.vehicle.mass_kg;
}

double PredictiveGuidanceController::computeMinQ(Dims::StateVec const& x0) const noexcept
{
    double const d_scale_sq = config_.softmin_config.d_scale * config_.softmin_config.d_scale;

    if (previous_state_trajectory_.empty())
    {
        return (x0.head<3>() - target_predictions_[0]).squaredNorm() / d_scale_sq;
    }

    double min_q = std::numeric_limits<double>::max();
    for (std::size_t k{0}; k < target_predictions_.size(); ++k)
    {
        double const q_k =
            (previous_state_trajectory_[k].head<3>() - target_predictions_[k]).squaredNorm() /
            d_scale_sq;
        min_q = std::min(min_q, q_k);
    }
    return min_q;
}

void PredictiveGuidanceController::predictTargetPositions(
    math::CartesianState const& target) noexcept
{
    for (std::size_t i{0}; i < target_predictions_.size(); ++i)
    {
        target_predictions_[i] =
            target.position_m + static_cast<double>(i) * config_.dt * target.velocity_mps;
    }
}

Dims::StateVec PredictiveGuidanceController::toAugmentedState(
    math::CartesianState const& interceptor) noexcept
{
    double const speed = interceptor.velocity_mps.norm();
    double const speed_safe = std::max(speed, min_speed_mps);
    double const psi = std::atan2(interceptor.velocity_mps.y(), interceptor.velocity_mps.x());
    double const gamma =
        std::asin(std::clamp(interceptor.velocity_mps.z() / speed_safe, -1.0, 1.0));

    Dims::StateVec x;
    x.head<3>() = interceptor.position_m;
    x[3] = speed;
    x[4] = psi;
    x[5] = gamma;
    x[6] = 0.0;
    return x;
}

}  // namespace guidance
