#pragma once

#include <cmath>
#include <cstddef>

namespace guidance
{

template <typename Scalar_T>
[[nodiscard]] Eigen::Vector<Scalar_T, 6> UAV3DofModel::operator()(
    Eigen::Vector<Scalar_T, 6> const& state,
    Eigen::Vector<Scalar_T, 3> const& control) const noexcept
{
    using std::cos;
    using std::max;
    using std::sin;

    constexpr double min_speed_mps{1.0e-3};
    constexpr double min_cos_gamma{1.0e-3};

    auto const v = state[3];
    auto const psi = state[4];
    auto const gamma = state[5];

    auto const thrust = control[0];
    auto const normal_load_factor = control[1];
    auto const bank_angle = control[2];

    auto const cos_gamma = cos(gamma);
    auto const sin_gamma = sin(gamma);
    auto const cos_psi = cos(psi);
    auto const sin_psi = sin(psi);
    auto const cos_bank = cos(bank_angle);
    auto const sin_bank = sin(bank_angle);

    auto const drag_force =
        0.5 * params.rho_kgpm3 * params.frontal_area_m2 * params.drag_coeff * v * v;

    // Protected division by zero
    auto const v_safe = max(v, min_speed_mps);
    auto const abs_cos_gamma = max(cos_gamma, -cos_gamma);
    auto const clamped_cos_gamma = max(abs_cos_gamma, min_cos_gamma);
    auto const cos_gamma_safe = (cos_gamma >= Scalar_T(0)) ? clamped_cos_gamma : -clamped_cos_gamma;

    auto const x_dot = v * cos_gamma * cos_psi;
    auto const y_dot = v * cos_gamma * sin_psi;
    auto const z_dot = v * sin_gamma;
    auto const v_dot = ((thrust - drag_force) / params.mass_kg) - math::gravity_mps2 * sin_gamma;
    auto const psi_dot =
        normal_load_factor * math::gravity_mps2 * sin_bank / (v_safe * cos_gamma_safe);
    auto const gamma_dot =
        math::gravity_mps2 * (normal_load_factor * cos_bank - cos_gamma) / v_safe;

    return Eigen::Vector<Scalar_T, 6>{x_dot, y_dot, z_dot, v_dot, psi_dot, gamma_dot};
}

template <typename Integrator_T, typename Diff_T>
AugmentedDiscreteUAV3DofModel<Integrator_T, Diff_T>::AugmentedDiscreteUAV3DofModel(
    UAV3DofModel uav_3dof_model, std::span<Eigen::Vector3d const> target_positions, double dt,
    SoftminConfig const& softmin_config, Integrator_T integrate, Diff_T differentiate) noexcept
    : uav_3dof_model_(std::move(uav_3dof_model)),
      dt_(dt),
      softmin_config_(softmin_config),
      target_positions_(target_positions),
      integrate_(std::move(integrate)),
      differentiate_(std::move(differentiate))
{
}

template <typename Integrator_T, typename Diff_T>
template <typename Scalar_T>
Eigen::Vector<Scalar_T, 7> AugmentedDiscreteUAV3DofModel<Integrator_T, Diff_T>::step(
    Eigen::Vector<Scalar_T, 7> const& x, Eigen::Vector<Scalar_T, 3> const& u, int k) const
{
    using std::exp;

    // Discretize the nominal 3dof UAV dynamics
    Eigen::Vector<Scalar_T, 6> const x_k = x.template head<6>();
    Eigen::Vector<Scalar_T, 6> const x_k_1 = integrate_(uav_3dof_model_, x_k, u, dt_);

    // Augment the state by the running log-sum-exp softmin
    auto const& target_position_k = target_positions_[static_cast<std::size_t>(k)];
    Scalar_T const q_k = ((target_position_k.x() - x[0]) * (target_position_k.x() - x[0]) +
                          (target_position_k.y() - x[1]) * (target_position_k.y() - x[1]) +
                          (target_position_k.z() - x[2]) * (target_position_k.z() - x[2])) /
                         (softmin_config_.d_scale * softmin_config_.d_scale);

    Scalar_T const z_k_1 = x[6] + exp(-softmin_config_.beta * (q_k - softmin_config_.min_q));

    Eigen::Vector<Scalar_T, 7> x_1;
    x_1.template head<6>() = x_k_1;
    x_1[6] = z_k_1;
    return x_1;
}

template <typename Integrator_T, typename Diff_T>
ilqr::DynamicsTaylorExpansion<typename AugmentedDiscreteUAV3DofModel<Integrator_T, Diff_T>::Dims>
AugmentedDiscreteUAV3DofModel<Integrator_T, Diff_T>::linearize(AugmentedStateVec const& x,
                                                               ControlVec const& u, int k) const
{
    auto discrete_step = [&, this](const auto& x_k, const auto& u_k)
    { return this->step(x_k, u_k, k); };

    const auto [A, B] = differentiate_(discrete_step, x, u);
    return {A, B};
}

}  // namespace guidance
