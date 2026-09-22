#include "guidance/predictive_guidance_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <ilqr/dynamics/autodiff_policy.hpp>
#include <ilqr/ilqr.hpp>
#include <iostream>
#include <limits>
#include <numbers>
#include <span>
#include <utility>

#include "guidance/cost.hpp"
#include "guidance/dynamics_model.hpp"
#include "ilqr/core/solver_io.hpp"
namespace guidance
{
namespace
{

/// @brief Floor on interceptor speed (m/s) applied before normalizing.
inline constexpr double min_speed_mps{1.0e-6};

/// @brief Floor on d_scale (m), applied so it can't collapse to zero as the interceptor closes
/// in on the target.
inline constexpr double min_d_scale_m{1.0};

using Model = AugmentedDiscreteUAV3DofModel<ilqr::math::HeunStep, ilqr::AutoDiff>;
using Cost = ilqr::CompositeCostFunction<SoftminCost, ilqr::ControlPenaltyCost<Dims>,
                                         ilqr::QuadraticStateRegulatorCost<Dims>>;
using Solver = ilqr::ILQRSolver<Model, Cost>;

}  // namespace

PredictiveGuidanceController::PredictiveGuidanceController(
    PredictiveGuidanceControllerConfig config)
    : config_(std::move(config)), target_predictions_(static_cast<std::size_t>(config_.horizon))
{
}

Eigen::Vector3d PredictiveGuidanceController::step(math::CartesianState const& target,
                                                   math::CartesianState const& interceptor, double)
{
    /// Compute internal model state out of cartesian state
    Dims::StateVec const x0 = toAugmentedState(interceptor);
    auto const& speed = x0[3];
    auto const& flight_path_angle_rad = x0[5];

    /// Boost phase: fixed climb until speed first reaches switch_speed_mps, then latched off
    /// permanently -- once out, we never revisit this branch, even if speed later dips again.
    if (!has_exited_boost_)
    {
        if (speed < config_.switch_speed_mps)
        {
            return Eigen::Vector3d{config_.max_thrust_n, 1.0, 0.0};
        }
        has_exited_boost_ = true;
    }

    /// Check if an interception exists from the buffer - If interception exists, shrink horizon to
    /// save compute and improve numerics
    double const interception_radius = speed * config_.dt;
    auto const interception_index = interceptionPredictedAt(interception_radius);
    int dynamic_horizon = config_.horizon;
    if (interception_index.has_value())
    {
        dynamic_horizon = std::min(interception_index.value() + 1, config_.horizon);
    }

    /// Resize buffer if horizon length changed
    if (size_t new_buffer_length = static_cast<size_t>(dynamic_horizon);
        new_buffer_length != target_predictions_.size())
    {
        target_predictions_.resize(new_buffer_length);
        previous_control_trajectory_.resize(new_buffer_length);
        previous_state_trajectory_.resize(new_buffer_length + 1);
    }

    /// Predict target positions
    predictTargetPositions(target);

    /// Scale d_scale to the current engagement distance so q_k stays well-conditioned regardless
    /// of how far away the target is (a fixed d_scale blows up for distant targets and collapses
    /// to zero right at intercept).
    double const d_scale = std::max((x0.head<3>() - target_predictions_[0]).norm(), min_d_scale_m);

    /// Compute softmin config - min_q is the minimal scaled squared distance between the current
    /// interceptor trajectory and the predicted target trajectory
    SoftminConfig const softmin_config{d_scale, computeMinQ(x0, d_scale),
                                       config_.softmin_config.beta};

    /// Create an UAV 3DoF model with an augmented state for softmin computation
    Model model(UAV3DofModel{config_.vehicle},
                std::span<Eigen::Vector3d const>(target_predictions_), config_.dt, softmin_config,
                ilqr::math::HeunStep{}, ilqr::AutoDiff{});

    /// Construct the cost function - (soft)min interception distance + control effort + cruise
    /// speed tracking
    SoftminCost softmin_cost(softmin_config, target_predictions_.back(), config_.softmin_weight);
    ilqr::ControlPenaltyCost<Dims> control_cost(
        Dims::ControlMat(config_.control_effort_weight.asDiagonal()));
    Dims::StateMat cruise_speed_Q = Dims::StateMat::Zero();
    cruise_speed_Q(3, 3) = config_.cruise_speed_weight;
    Dims::StateVec cruise_speed_ref = Dims::StateVec::Zero();
    cruise_speed_ref[3] = config_.switch_speed_mps;
    ilqr::QuadraticStateRegulatorCost<Dims> cruise_speed_cost(cruise_speed_Q, cruise_speed_ref);
    Cost cost(std::move(softmin_cost), std::move(control_cost), std::move(cruise_speed_cost));

    /// Construct solver
    Solver solver(std::move(model), std::move(cost), config_.solver_config);

    /// Construct limits. Boost is handled entirely by the early return above, so once we reach
    /// here we are always past it -- thrust is always bounded by the trim value, never max.
    Dims::ControlVec const lower{0.0, config_.transverse_limits.min_load_factor,
                                 -config_.transverse_limits.max_bank_angle_rad};
    Dims::ControlVec const upper{trimThrust(speed, flight_path_angle_rad),
                                 config_.transverse_limits.max_load_factor,
                                 config_.transverse_limits.max_bank_angle_rad};

    /// Cold-start if this is the first solve ever (nothing to warm-start from yet), otherwise
    /// warm-start from the previous solve's trajectory.
    auto const request =
        previous_control_trajectory_.empty()
            ? ilqr::SolveRequest<Dims>::cold_start(x0, dynamic_horizon)
                  .with_control_bounds(lower, upper)
            : ilqr::SolveRequest<Dims>::warm_start(x0, previous_control_trajectory_)
                  .with_control_bounds(lower, upper);

    /// Solve
    auto const result = solver.solve(request);

    /// On failure (InvalidProblem/BoxQPFailed/MaxRegularization) the trajectory is empty; keep
    /// the existing warm start and fallback either to the previous or the boost control
    if (result.status != ilqr::SolverStatus::Converged &&
        result.status != ilqr::SolverStatus::MaxIterations)
    {
        if (previous_control_trajectory_.empty())
        {
            return Eigen::Vector3d{trimThrust(speed, flight_path_angle_rad), 1.0, 0.0};
        }
        return previous_control_trajectory_[0];
    }

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

double PredictiveGuidanceController::computeMinQ(Dims::StateVec const& x0,
                                                 double d_scale) const noexcept
{
    double const d_scale_sq = d_scale * d_scale;

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

std::optional<int> PredictiveGuidanceController::interceptionPredictedAt(
    double interception_radius) noexcept
{
    if (target_predictions_.empty() || previous_state_trajectory_.empty())
    {
        return std::nullopt;
    }

    double const interception_radius_sq = interception_radius * interception_radius;
    for (std::size_t k{0}; k < target_predictions_.size(); ++k)
    {
        double const dist_sq =
            (previous_state_trajectory_[k].head<3>() - target_predictions_[k]).squaredNorm();
        if (dist_sq <= interception_radius_sq)
        {
            return static_cast<int>(k);
        }
    }
    return std::nullopt;
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
