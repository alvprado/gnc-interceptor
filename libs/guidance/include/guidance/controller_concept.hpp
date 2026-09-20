#pragma once

#include <concepts>
#include <Eigen/Dense>

#include "math/cartesian_state.hpp"

namespace guidance
{

/// @brief A control law mapping target and interceptor kinematics to a
/// commanded control, interchangeable across guidance strategies (e.g. PN,
/// iLQR).
/// @details step() returns [thrust, load_factor, bank_angle_rad]
/// @tparam Controller_T The controller type to check.
template <typename Controller_T>
concept GuidanceController = requires(Controller_T& controller, math::CartesianState const& target,
                                      math::CartesianState const& interceptor, double dt) {
                                 {
                                     controller.step(target, interceptor, dt)
                                 } -> std::convertible_to<Eigen::Vector3d>;
                             };

}  // namespace guidance
