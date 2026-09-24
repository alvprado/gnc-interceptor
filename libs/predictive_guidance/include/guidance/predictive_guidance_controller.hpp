#pragma once

#include <Eigen/Dense>
#include <ilqr/core/config.hpp>
#include <ilqr/core/types.hpp>
#include <numbers>

#include "guidance/controller_concept.hpp"
#include "guidance/dynamics_model.hpp"
#include "guidance/thrust_control_law.hpp"
#include "guidance/transverse_control_allocation.hpp"
#include "math/cartesian_state.hpp"
#include "math/constants.hpp"

namespace guidance
{

/// @brief Configuration for PredictiveGuidanceController.
struct PredictiveGuidanceControllerConfig
{
    int horizon{100};    ///< Number of initial and maximal stages in the iLQR horizon.
    int min_horizon{5};  ///< Lower bound on horizon length
    double dt{0.1};      ///< Integration/prediction timestep.
    ThrustControlConfig thrust_control{};  ///< Boost/trim thresholds and vehicle model for
                                           ///< thrust, decoupled from the transverse solve.
    TransverseControlAllocationConfig transverse_limits{};  ///< Load factor / bank angle bounds
    Dims::ControlVec control_effort_weight{
        1.0 / (9.0 * 9.0),
        1.0 / (std::numbers::pi *
               std::numbers::pi)};           ///< Control rate weights (load factor and bank)
    double final_interception_weight{10.0};  ///< Weight on the final interception error
    double running_interception_weight{
        10.0};  ///< Weight on the interception error accross the horizon
    ilqr::SolverConfig<double> solver_config{};  ///< iLQR iteration/regularization tuning
};

/// @brief Predictive guidance controller: builds and solves an iLQR problem over load factor and
/// bank angle from the current target and interceptor state each update, and returns its optimal
/// control alongside a thrust command from a decoupled boost/trim law.
class PredictiveGuidanceController
{
public:
    /// @brief Construct from configuration.
    /// @param[in] config The boost/trim thresholds, horizon and cost tuning.
    explicit PredictiveGuidanceController(PredictiveGuidanceControllerConfig config);

    /// @brief Compute the commanded control for one guidance update.
    /// @param[in] target The target's Cartesian state, used to predict its trajectory over the
    ///        horizon under a constant-velocity assumption.
    /// @param[in] interceptor The interceptor's Cartesian state.
    /// @returns [thrust, load_factor, bank_angle_rad].
    [[nodiscard]] Eigen::Vector3d step(math::CartesianState const& target,
                                       math::CartesianState const& interceptor, double);

private:
    /// @brief Refresh target_predictions_ in place with the target's future positions at
    /// t = 0, dt, 2*dt, ..., over the horizon, extrapolated under constant acceleration.
    /// @param[in] target The target's Cartesian state.
    void predictTargetPositions(math::CartesianState const& target) noexcept;

    /// @brief Convert the interceptor's Cartesian state to the augmented model state, with the
    /// previously-commanded (load factor, bank) as the state's tail.
    /// @param[in] interceptor The interceptor's Cartesian state.
    /// @param[in] previous_control The (load factor, bank) commanded on the prior step.
    [[nodiscard]] static Dims::StateVec toModelState(
        math::CartesianState const& interceptor, Dims::ControlVec const& previous_control) noexcept;

    /// @brief Estimate the horizon length (in stages) from the analytic time to closest approach,
    /// assuming the interceptor holds its current speed and turns instantaneously.
    /// @param[in] target The target's Cartesian state.
    /// @param[in] interceptor The interceptor's Cartesian state.
    /// @returns The stage count, clamped to [min_horizon, horizon].
    int computeHorizonLenght(math::CartesianState const& target,
                             math::CartesianState const& interceptor) noexcept;

    PredictiveGuidanceControllerConfig config_;
    ThrustControlLaw thrust_control_law_;
    ilqr::AlignedVec<Eigen::Vector3d> target_predictions_;
    ilqr::AlignedVec<Dims::ControlVec> previous_control_trajectory_;
    ilqr::AlignedVec<Dims::StateVec> previous_state_trajectory_;
    bool has_exited_boost_{false};  ///< Latches true the first time boost is exited; never reset.
};

static_assert(GuidanceController<PredictiveGuidanceController>);

}  // namespace guidance
