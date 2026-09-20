#include "simulation/uav_3dof_model.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "math/angles.hpp"
#include "math/constants.hpp"

namespace simulation
{
namespace
{

/// @brief Floor on |cos(gamma)| applied to the heading-rate denominator.
inline constexpr double min_cos_gamma{1.0e-2};

}  // namespace

UAV3DofModel::UAV3DofModel(UAV3DofModelParams const& params, UAV3DofModelLimits const& limits)
    : params_(params), limits_(limits)
{
}

UAV3DofModel::StateVec UAV3DofModel::operator()(StateVec const& state,
                                                ControlVec const& control) const noexcept
{
    auto const v = state[3];
    auto const psi = state[4];
    auto const gamma = state[5];

    auto const thrust = control[0];
    auto const normal_load_factor = control[1];
    auto const bank_angle = control[2];

    auto const cos_gamma = std::cos(gamma);
    auto const sin_gamma = std::sin(gamma);
    auto const cos_psi = std::cos(psi);
    auto const sin_psi = std::sin(psi);
    auto const cos_bank = std::cos(bank_angle);
    auto const sin_bank = std::sin(bank_angle);

    auto const drag_force =
        0.5 * params_.rho_kgpm3 * params_.frontal_area_m2 * params_.drag_coeff * v * v;

    // Protected division by zero; copysign keeps the turn direction
    // correct for a flight-path angle past vertical.
    auto const v_safe = std::max(v, limits_.min_speed_mps);
    auto const cos_gamma_safe =
        std::copysign(std::max(std::abs(cos_gamma), min_cos_gamma), cos_gamma);

    auto const x_dot = v * cos_gamma * cos_psi;
    auto const y_dot = v * cos_gamma * sin_psi;
    auto const z_dot = v * sin_gamma;
    auto const v_dot = ((thrust - drag_force) / params_.mass_kg) - math::gravity_mps2 * sin_gamma;
    auto const psi_dot =
        normal_load_factor * math::gravity_mps2 * sin_bank / (v_safe * cos_gamma_safe);
    auto const gamma_dot =
        math::gravity_mps2 * (normal_load_factor * cos_bank - cos_gamma) / v_safe;

    return StateVec{x_dot, y_dot, z_dot, v_dot, psi_dot, gamma_dot};
}

math::CartesianState UAV3DofModel::toCartesianState(StateVec const& state) const noexcept
{
    auto const v = state[3];
    auto const psi = state[4];
    auto const gamma = state[5];
    auto const cos_gamma = std::cos(gamma);

    math::CartesianState cartesian;
    cartesian.position_m = state.head<3>();
    cartesian.velocity_mps = Eigen::Vector3d{v * cos_gamma * std::cos(psi),
                                              v * cos_gamma * std::sin(psi), v * std::sin(gamma)};
    return cartesian;
}

UAV3DofModel::StateVec UAV3DofModel::fromCartesianState(
    math::CartesianState const& cartesian) const noexcept
{
    auto const& velocity = cartesian.velocity_mps;
    auto const v = velocity.norm();
    auto const psi = std::atan2(velocity.y(), velocity.x());
    auto const gamma = (v > 0.0) ? std::asin(std::clamp(velocity.z() / v, -1.0, 1.0)) : 0.0;

    StateVec state;
    state.head<3>() = cartesian.position_m;
    state[3] = v;
    state[4] = psi;
    state[5] = gamma;
    return state;
}

UAV3DofModel::ControlVec UAV3DofModel::clampControl(ControlVec const& control) const noexcept
{
    return ControlVec{
        std::clamp(control[0], limits_.min_thrust_n, limits_.max_thrust_n),
        std::clamp(control[1], limits_.min_load_factor, limits_.max_load_factor),
        std::clamp(control[2], -limits_.max_bank_angle_rad, limits_.max_bank_angle_rad)};
}

UAV3DofModel::StateVec UAV3DofModel::clampState(StateVec const& state) const noexcept
{
    StateVec clamped = state;
    clamped[3] = std::clamp(state[3], limits_.min_speed_mps, limits_.max_speed_mps);
    clamped[4] = math::wrapToPi(state[4]);
    clamped[5] = std::clamp(state[5], -std::numbers::pi / 2.0, std::numbers::pi / 2.0);
    return clamped;
}

}  // namespace simulation
