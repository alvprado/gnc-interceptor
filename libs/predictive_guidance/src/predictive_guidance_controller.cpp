#include "guidance/predictive_guidance_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <ilqr/dynamics/autodiff_policy.hpp>
#include <ilqr/ilqr.hpp>
#include <utility>

#include "guidance/control_rate_cost_term.hpp"
#include "guidance/dynamics_model.hpp"

namespace guidance
{
namespace
{

inline constexpr double min_speed_mps{1.0e-6};  ///< avoids div-by-zero when normalizing velocity
inline constexpr double min_d_scale_m{1.0};     ///< avoids div-by-zero as range collapses to 0

}  // namespace

PredictiveGuidanceController::PredictiveGuidanceController(
    PredictiveGuidanceControllerConfig config)
    : config_(std::move(config)),
      thrust_control_law_(config_.thrust_control),
      target_predictions_(static_cast<std::size_t>(config_.horizon))
{
}

Eigen::Vector3d PredictiveGuidanceController::step(math::CartesianState const& target,
                                                   math::CartesianState const& interceptor, double)
{
    double const speed = interceptor.velocity_mps.norm();

    if (!has_exited_boost_)
    {
        if (speed < thrust_control_law_.switchSpeedMps())
        {
            return Eigen::Vector3d{thrust_control_law_.step(interceptor), 1.0, 0.0};
        }
        has_exited_boost_ = true;
    }

    Dims::ControlVec const previous_control = previous_control_trajectory_.empty()
                                                  ? Dims::ControlVec{1.0, 0.0}
                                                  : previous_control_trajectory_[0];
    Dims::StateVec const x0 = toModelState(interceptor, previous_control);

    int const dynamic_horizon = computeHorizonLenght(target, interceptor);

    if (size_t new_buffer_length = static_cast<size_t>(dynamic_horizon);
        new_buffer_length != target_predictions_.size())
    {
        target_predictions_.resize(new_buffer_length);
        previous_control_trajectory_.resize(new_buffer_length);
        previous_state_trajectory_.resize(new_buffer_length + 1);
    }

    predictTargetPositions(target);

    AugmentedDiscreteUAV3DofModel<ilqr::math::HeunStep, ilqr::AutoDiff> model(
        UAV3DofModel{config_.thrust_control.vehicle, speed}, config_.dt, ilqr::math::HeunStep{},
        ilqr::AutoDiff{});

    double const d_scale = std::max((x0.head<3>() - target_predictions_[0]).norm(), min_d_scale_m);
    Dims::StateMat R_interception = Dims::StateMat::Zero();
    R_interception.diagonal().head<3>().setConstant(config_.final_interception_weight /
                                                    (d_scale * d_scale));
    Dims::StateVec x_goal = Dims::StateVec::Zero();
    x_goal.head<3>() = target_predictions_.back();
    ilqr::FinalCost<Dims> interception_cost(R_interception, x_goal);
    ControlRateCost<Dims> control_rate_cost(
        Dims::ControlMat(config_.control_effort_weight.asDiagonal()));

    Dims::StateMat R_tracking = Dims::StateMat::Zero();
    R_tracking.diagonal().head<3>().setConstant(config_.running_interception_weight /
                                                (d_scale * d_scale));
    ilqr::AlignedVec<Dims::StateVec> tracking_ref(target_predictions_.size());
    for (std::size_t k{0}; k < target_predictions_.size(); ++k)
    {
        tracking_ref[k] = Dims::StateVec::Zero();
        tracking_ref[k].head<3>() = target_predictions_[k];
    }
    ilqr::QuadraticTrackingCost<Dims> tracking_cost(R_tracking, std::move(tracking_ref));

    ilqr::CompositeCostFunction cost(std::move(interception_cost), std::move(control_rate_cost),
                                     std::move(tracking_cost));

    ilqr::ILQRSolver solver{std::move(model), std::move(cost), config_.solver_config};

    Dims::ControlVec const lower{config_.transverse_limits.min_load_factor,
                                 -config_.transverse_limits.max_bank_angle_rad};
    Dims::ControlVec const upper{config_.transverse_limits.max_load_factor,
                                 config_.transverse_limits.max_bank_angle_rad};

    auto const request =
        previous_control_trajectory_.empty()
            ? ilqr::SolveRequest<Dims>::cold_start(x0, dynamic_horizon)
                  .with_control_bounds(lower, upper)
            : ilqr::SolveRequest<Dims>::warm_start(x0, previous_control_trajectory_)
                  .with_control_bounds(lower, upper);

    auto const result = solver.solve(request);

    if (result.status != ilqr::SolverStatus::Converged &&
        result.status != ilqr::SolverStatus::MaxIterations)
    {
        return Eigen::Vector3d{thrust_control_law_.trimThrust(interceptor), previous_control[0],
                               previous_control[1]};
    }

    previous_control_trajectory_.assign(result.trajectory.controls().begin(),
                                        result.trajectory.controls().end());
    previous_state_trajectory_.assign(result.trajectory.states().begin(),
                                      result.trajectory.states().end());

    Dims::ControlVec const u0 = result.trajectory.control(0);
    return Eigen::Vector3d{thrust_control_law_.trimThrust(interceptor), u0[0], u0[1]};
}

int PredictiveGuidanceController::computeHorizonLenght(
    math::CartesianState const& target, math::CartesianState const& interceptor) noexcept
{
    const double a = target.velocity_mps.squaredNorm() - interceptor.velocity_mps.squaredNorm();
    if (a >= 0.0)
    {
        return config_.horizon;
    }

    Eigen::Vector3d const relative_position = target.position_m - interceptor.position_m;

    const double b = 2 * relative_position.dot(target.velocity_mps);
    const double c = relative_position.squaredNorm();
    const double time_to_collision = (-b - std::sqrt(b * b - 4 * a * c)) / (2 * a);

    if (time_to_collision < 0.0)
    {
        return config_.horizon;
    }

    return std::clamp(static_cast<int>(std::ceil(time_to_collision / config_.dt)) + 1,
                      config_.min_horizon, config_.horizon);
}

void PredictiveGuidanceController::predictTargetPositions(
    math::CartesianState const& target) noexcept
{
    for (std::size_t i{0}; i < target_predictions_.size(); ++i)
    {
        double const time = i * config_.dt;
        target_predictions_[i] = target.position_m + time * target.velocity_mps +
                                 0.5 * time * time * target.acceleration_mps2;
    }
}

Dims::StateVec PredictiveGuidanceController::toModelState(
    math::CartesianState const& interceptor, Dims::ControlVec const& previous_control) noexcept
{
    double const speed = interceptor.velocity_mps.norm();
    double const speed_safe = std::max(speed, min_speed_mps);
    double const psi = std::atan2(interceptor.velocity_mps.y(), interceptor.velocity_mps.x());
    double const gamma =
        std::asin(std::clamp(interceptor.velocity_mps.z() / speed_safe, -1.0, 1.0));

    Dims::StateVec x;
    x.head<3>() = interceptor.position_m;
    x[3] = psi;
    x[4] = gamma;
    x.tail<2>() = previous_control;
    return x;
}

}  // namespace guidance
