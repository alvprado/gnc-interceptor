#pragma once

#include <cmath>
#include <Eigen/Dense>
#include <ilqr/core/types.hpp>
#include <ilqr/dynamics/dynamics_concepts.hpp>
#include <span>

#include "guidance/model_parameters.hpp"

namespace guidance
{

/// @brief Continuous-time 3DoF vehicle dynamics ẋ = f(x, u).
struct UAV3DofModel
{
    /// @brief Evaluate the continuous dynamics ẋ = f(x, u).
    /// @tparam Scalar_T The scalar type to evaluate the dynamics at.
    /// @param[in] state The state to evaluate the dynamics at.
    /// @param[in] control The control to evaluate the dynamics at.
    /// @returns The state derivative ẋ.
    template <typename Scalar_T>
    [[nodiscard]] Eigen::Vector<Scalar_T, 6> operator()(
        Eigen::Vector<Scalar_T, 6> const& state,
        Eigen::Vector<Scalar_T, 3> const& control) const noexcept;

    /// Vehicle physical parameters.
    ModelParameters params;
};

/// @brief Discrete-time UAV3DofModel augmented with a running log-sum-exp softmin of the
/// distance to a fixed target trajectory, for use as an iLQR Dynamics model.
/// @tparam Integrator_T The integration policy advancing the vehicle dynamics by one step.
/// @tparam Diff_T The differentiation policy used to linearize the discrete step.
template <typename Integrator_T, typename Diff_T>
class AugmentedDiscreteUAV3DofModel
{
public:
    /// Aliases
    using Dims = ilqr::Dims<7, 3, double>;
    using AugmentedStateVec = Eigen::Vector<double, 7>;
    using ControlVec = Eigen::Vector<double, 3>;

    /// @brief Construct from a UAV 3DoF model and its fixed target-position predictions.
    /// @param[in] uav_3dof_model The continuous vehicle dynamics to discretize.
    /// @param[in] target_positions The predicted target position at each stage of the horizon.
    /// @param[in] dt The integration timestep.
    /// @param[in] integrate_ The integration policy advancing the vehicle dynamics.
    /// @param[in] differentiate_ The differentiation policy used by linearize().
    AugmentedDiscreteUAV3DofModel(UAV3DofModel uav_3dof_model,
                                  std::span<Eigen::Vector3d const> target_positions, double dt,
                                  Integrator_T integrate_, Diff_T differentiate_) noexcept;

    /// @brief Discrete dynamics x_{k+1} = f_d(x_k, u_k, k), obtained by integrating the vehicle
    /// model and accumulating the log-sum-exp softmin term.
    /// @tparam Scalar_T The scalar type to evaluate the step at.
    /// @param[in] x The augmented state [vehicle state, running softmin accumulator]^T.
    /// @param[in] u The control.
    /// @param[in] k The stage index, used to look up the target position prediction.
    /// @returns The augmented state advanced by one timestep.
    template <typename Scalar_T>
    [[nodiscard]] Eigen::Vector<Scalar_T, 7> step(Eigen::Vector<Scalar_T, 7> const& x,
                                                  Eigen::Vector<Scalar_T, 3> const& u, int k) const
    {
        using std::exp;

        // Discretize the nominal 3dof UAV dynamics
        Eigen::Vector<Scalar_T, 6> const x_k = x.template head<6>();
        Eigen::Vector<Scalar_T, 6> const x_k_1 = integrate_(uav_3dof_model_, x_k, u, dt_);

        // Augment the state by the running log-sum-exp softmin
        auto const& target_position_k = target_positions_[k];
        Scalar_T const q_k = ((target_position_k.x() - x[0]) * (target_position_k.x() - x[0]) +
                              (target_position_k.y() - x[1]) * (target_position_k.y() - x[1]) +
                              (target_position_k.z() - x[2]) * (target_position_k.z() - x[2])) /
                             (d_scale_ * d_scale_);

        Scalar_T const z_k_1 = x[6] + exp(-beta_ * (q_k - min_q_));

        return Eigen::Vector<Scalar_T, 7>{x_k_1, z_k_1};
    }

    /// @brief Jacobians A = ∂f_d/∂x, B = ∂f_d/∂u of the discrete step, via the differentiation
    ///        policy.
    /// @param[in] x The augmented state to linearize about.
    /// @param[in] u The control to linearize about.
    /// @param[in] k The stage index, used to look up the target position prediction.
    /// @returns The first-order Taylor expansion of step() at (x, u, k).
    [[nodiscard]] ilqr::DynamicsTaylorExpansion<Dims> linearize(AugmentedStateVec const& x,
                                                                ControlVec const& u, int k) const
    {
        auto discrete_step = [&, this](const auto& x_k, const auto& u_k)
        { return this->step(x_k, u_k, k); };

        const auto [A, B] = differentiate_(discrete_step, x, u);
        return {A, B};
    }

private:
    UAV3DofModel uav_3dof_model_;                         ///< Continuous vehicle dynamics.
    double dt_;                                           ///< Integration timestep.
    double d_scale_{10.0};                                ///< Distance normalization for q_k.
    double min_q_{1.0};                                   ///< Numerical-stability shift for z_k_1.
    double beta_{2.0};                                    ///< Softmin sharpness.
    std::span<Eigen::Vector3d const> target_positions_;   ///< Predicted target position per stage.
    [[no_unique_address]] Integrator_T integrate_;         ///< Integration policy.
    [[no_unique_address]] Diff_T differentiate_;           ///< Differentiation policy.
};
}  // namespace guidance