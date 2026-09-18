#pragma once

#include <Eigen/Dense>

#include <concepts>

namespace target
{

/// @brief Ground-truth state of the target at an instant.
struct TargetState
{
    Eigen::Vector3d position_m{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity_mps{Eigen::Vector3d::Zero()};
    Eigen::Vector3d acceleration_mps2{Eigen::Vector3d::Zero()};
};

/// @brief A closed-form target trajectory evaluated at an arbitrary time.
/// @details Evaluation is a pure function of time, so the trajectory is exact,
/// reproducible and free of integration drift, and may be sampled off the
/// simulation grid.
/// @tparam Trajectory_T The trajectory type to check.
template <typename Trajectory_T>
concept TargetTrajectory = requires(Trajectory_T const& trajectory, double time_s) {
                               {
                                   trajectory.evaluateTargetStateAt(time_s)
                               } -> std::convertible_to<TargetState>;
                           };

}  // namespace target
