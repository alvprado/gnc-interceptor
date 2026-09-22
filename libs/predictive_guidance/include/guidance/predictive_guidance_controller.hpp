#pragma once

#include <cmath>
#include <Eigen/Dense>
#include <ilqr/core/config.hpp>
#include <ilqr/core/types.hpp>

#include "guidance/controller_concept.hpp"
#include "guidance/model_parameters.hpp"
#include "guidance/transverse_control_allocation.hpp"
#include "guidance/types.hpp"
#include "math/cartesian_state.hpp"
#include "math/constants.hpp"

namespace guidance
{

/// @brief Configuration for PredictiveGuidanceController.
struct PredictiveGuidanceControllerConfig
{
    double switch_speed_mps{90.0};  ///< Speed for switching from boost to trim.
    double max_thrust_n{150.0};     ///< Thrust commanded during boost, in N.
    int horizon{50};                ///< Number of stages in the iLQR horizon.
    double dt{0.1};                 ///< Integration/prediction timestep.
    ModelParameters vehicle{};      ///< Vehicle model used by the dynamics and the trim bound.
    TransverseControlAllocationConfig transverse_limits{};  ///< Load factor / bank angle bounds.
    SoftminConfig softmin_config{10.0, 1.0, 2.0};           ///< Softmin parameters
    Dims::ControlVec control_effort_weight{
        Dims::ControlVec::Ones()};               ///< Diagonal control-effort weights [w_thrust,
                                                 ///< w_load_factor,w_bank_angle];
    ilqr::SolverConfig<double> solver_config{};  ///< iLQR iteration/regularization tuning.
};

/// @brief Predictive guidance controller: builds and solves an iLQR problem from the current
/// target and interceptor state each update, and returns its optimal first control.
/// @details Below switch_speed_mps, the vehicle is held wings-level through boost (n=1, bank=0,
/// thrust=max_thrust_n) and no solve is run. Above it, thrust is bounded up to the trim thrust
/// (drag + gravity compensation) for the current state, and the solver optimizes thrust, load
/// factor and bank within that and the configured transverse limits.
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
    /// @returns [thrust, load_factor, bank_angle_rad], the solver's optimal first control.
    [[nodiscard]] Eigen::Vector3d step(math::CartesianState const& target,
                                       math::CartesianState const& interceptor, double);

private:
    /// @brief Feedforward thrust that exactly cancels drag and the gravity component along the
    /// flight path (v_dot = 0): the trim thrust to hold speed in level or climbing/descending
    /// flight.
    /// @param[in] speed_mps The vehicle's speed.
    /// @param[in] flight_path_angle_rad The flight-path angle (elevation of the velocity vector).
    /// @returns The trim thrust, in N.
    [[nodiscard]] double trimThrust(double speed_mps, double flight_path_angle_rad) const noexcept;

    /// @brief Refresh target_predictions_ in place with the target's future positions assuming
    /// constant velocity, at t = 0, dt, 2*dt, ..., over the horizon.
    /// @param[in] target The target's Cartesian state.
    void predictTargetPositions(math::CartesianState const& target) noexcept;

    /// @brief Recompute min_q as the minimum squared, d_scale-normalized distance between the
    /// nominal trajectory and target_predictions_ over the horizon.
    /// @details The nominal trajectory is the previous call's solved state trajectory; before a
    /// first solve exists, only the initial distance (x0 to target_predictions_[0]) is used.
    /// @param[in] x0 The current augmented initial state.
    /// @returns The recomputed min_q.
    [[nodiscard]] double computeMinQ(Dims::StateVec const& x0) const noexcept;

    /// @brief Convert the interceptor's Cartesian state to the augmented native state, with a
    ///        zero-initialized softmin accumulator.
    [[nodiscard]] static Dims::StateVec toAugmentedState(
        math::CartesianState const& interceptor) noexcept;

    PredictiveGuidanceControllerConfig config_;
    ilqr::AlignedVec<Eigen::Vector3d> target_predictions_;
    ilqr::AlignedVec<Dims::ControlVec> previous_control_trajectory_;
    ilqr::AlignedVec<Dims::StateVec> previous_state_trajectory_;
};

static_assert(GuidanceController<PredictiveGuidanceController>);

}  // namespace guidance
