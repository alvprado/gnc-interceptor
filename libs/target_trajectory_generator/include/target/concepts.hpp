#pragma once

#include "math/cartesian_state.hpp"

#include <concepts>

namespace target
{

/// @brief A closed-form target trajectory evaluated at an arbitrary time.
/// @details Evaluation is a pure function of time, so the trajectory is exact,
/// reproducible and free of integration drift, and may be sampled off the
/// simulation grid.
/// @tparam Trajectory_T The trajectory type to check.
template <typename Trajectory_T>
concept TargetTrajectory = requires(Trajectory_T const& trajectory, double time_s) {
                               {
                                   trajectory.evaluateTargetStateAt(time_s)
                               } -> std::convertible_to<math::CartesianState>;
                           };

}  // namespace target
