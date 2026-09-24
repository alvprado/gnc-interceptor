#pragma once

#include <cmath>
#include <cstddef>

namespace guidance
{

template <typename Scalar_T>
[[nodiscard]] Eigen::Vector<Scalar_T, 5> UAV3DofModel::operator()(
    Eigen::Vector<Scalar_T, 5> const& state,
    Eigen::Vector<Scalar_T, 2> const& control) const noexcept
{
    using std::cos;
    using std::max;
    using std::sin;

    constexpr double min_speed_mps{1.0e-3};
    constexpr double min_cos_gamma{1.0e-3};

    auto const psi = state[3];
    auto const gamma = state[4];

    auto const normal_load_factor = control[0];
    auto const bank_angle = control[1];

    auto const cos_gamma = cos(gamma);
    auto const sin_gamma = sin(gamma);
    auto const cos_psi = cos(psi);
    auto const sin_psi = sin(psi);
    auto const cos_bank = cos(bank_angle);
    auto const sin_bank = sin(bank_angle);

    // Protected division by zero
    auto const v_safe = max(speed_mps, min_speed_mps);
    auto const abs_cos_gamma = max(cos_gamma, -cos_gamma);
    auto const clamped_cos_gamma = max(abs_cos_gamma, min_cos_gamma);
    auto const cos_gamma_safe = (cos_gamma >= Scalar_T(0)) ? clamped_cos_gamma : -clamped_cos_gamma;

    auto const x_dot = speed_mps * cos_gamma * cos_psi;
    auto const y_dot = speed_mps * cos_gamma * sin_psi;
    auto const z_dot = speed_mps * sin_gamma;
    auto const psi_dot =
        normal_load_factor * math::gravity_mps2 * sin_bank / (v_safe * cos_gamma_safe);
    auto const gamma_dot =
        math::gravity_mps2 * (normal_load_factor * cos_bank - cos_gamma) / v_safe;

    return Eigen::Vector<Scalar_T, 5>{x_dot, y_dot, z_dot, psi_dot, gamma_dot};
}

template <typename Integrator_T, typename Diff_T>
AugmentedDiscreteUAV3DofModel<Integrator_T, Diff_T>::AugmentedDiscreteUAV3DofModel(
    UAV3DofModel uav_3dof_model, double dt, Integrator_T integrate,
    Diff_T differentiate) noexcept
    : uav_3dof_model_(std::move(uav_3dof_model)),
      dt_(dt),
      integrate_(std::move(integrate)),
      differentiate_(std::move(differentiate))
{
}

template <typename Integrator_T, typename Diff_T>
template <typename Scalar_T>
Eigen::Vector<Scalar_T, 7> AugmentedDiscreteUAV3DofModel<Integrator_T, Diff_T>::step(
    Eigen::Vector<Scalar_T, 7> const& x, Eigen::Vector<Scalar_T, 2> const& u, int k) const
{
    using std::exp;

    // Discretize the nominal 3dof UAV dynamics
    Eigen::Vector<Scalar_T, 5> const x_k = x.template head<5>();
    Eigen::Vector<Scalar_T, 5> const x_k_1 = integrate_(uav_3dof_model_, x_k, u, dt_);

    // Augment the state by control input
    Eigen::Vector<Scalar_T, 7> x_aug;
    x_aug.template head<5>() = x_k_1;
    x_aug.template tail<2>() = u;
    return x_aug;
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
