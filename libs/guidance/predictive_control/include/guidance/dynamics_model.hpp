#pragma once

#include <Eigen/Dense>
#include <ilqr/dynamics/dynamics_concepts.hpp>
#include <span>

#include "guidance/model_parameters.hpp"

namespace guidance
{

/// Dims
using Dims = ilqr::Dims<7, 2, double>;

/// @brief Continuous-time 3DoF vehicle dynamics ẋ = f(x, u).
struct UAV3DofModel
{
    /// @brief Evaluate the continuous dynamics ẋ = f(x, u).
    /// @tparam Scalar_T The scalar type to evaluate the dynamics at.
    /// @param[in] state The state to evaluate the dynamics at.
    /// @param[in] control The control to evaluate the dynamics at.
    /// @returns The state derivative ẋ.
    template <typename Scalar_T>
    [[nodiscard]] Eigen::Vector<Scalar_T, 5> operator()(
        Eigen::Vector<Scalar_T, 5> const& state,
        Eigen::Vector<Scalar_T, 2> const& control) const noexcept;

    /// Vehicle physical parameters.
    ModelParameters params;

    /// Vehicle speed
    double speed_mps;
};

/// @brief Discrete-time UAV3DofModel augmented with the previously-applied control
/// @tparam Integrator_T The integration policy advancing the vehicle dynamics by one step.
/// @tparam Diff_T The differentiation policy used to linearize the discrete step.
template <typename Integrator_T, typename Diff_T>
class AugmentedDiscreteUAV3DofModel
{
public:
    /// Aliases
    using Dims = guidance::Dims;
    using AugmentedStateVec = Eigen::Vector<double, 7>;
    using ControlVec = Eigen::Vector<double, 2>;

    /// @brief Construct from a UAV 3DoF model.
    /// @param[in] uav_3dof_model The continuous vehicle dynamics to discretize.
    /// @param[in] dt The integration timestep.
    /// @param[in] integrate The integration policy advancing the vehicle dynamics.
    /// @param[in] differentiate The differentiation policy used by linearize().
    AugmentedDiscreteUAV3DofModel(UAV3DofModel uav_3dof_model, double dt, Integrator_T integrate,
                                  Diff_T differentiate) noexcept;

    /// @brief Discrete dynamics x_{k+1} = f_d(x_k, u_k, k), obtained by integrating the vehicle
    /// model and carrying u_k forward as the state's previous-control tail.
    /// @tparam Scalar_T The scalar type to evaluate the step at.
    /// @param[in] x The augmented state [vehicle state, previous control]^T.
    /// @param[in] u The control.
    /// @param[in] k Unused; the wrapped model is time-invariant.
    /// @returns The augmented state advanced by one timestep.
    template <typename Scalar_T>
    [[nodiscard]] Eigen::Vector<Scalar_T, 7> step(Eigen::Vector<Scalar_T, 7> const& x,
                                                  Eigen::Vector<Scalar_T, 2> const& u, int k) const;

    /// @brief Jacobians A = ∂f_d/∂x, B = ∂f_d/∂u of the discrete step, via the differentiation
    ///        policy.
    /// @param[in] x The augmented state to linearize about.
    /// @param[in] u The control to linearize about.
    /// @param[in] k Unused; the wrapped model is time-invariant.
    /// @returns The first-order Taylor expansion of step() at (x, u, k).
    [[nodiscard]] ilqr::DynamicsTaylorExpansion<Dims> linearize(AugmentedStateVec const& x,
                                                                ControlVec const& u, int k) const;

private:
    UAV3DofModel uav_3dof_model_;                   ///< Continuous vehicle dynamics.
    double dt_;                                     ///< Integration timestep.
    [[no_unique_address]] Integrator_T integrate_;  ///< Integration policy.
    [[no_unique_address]] Diff_T differentiate_;    ///< Differentiation policy.
};

}  // namespace guidance

#include "guidance/dynamics_model.inl"
