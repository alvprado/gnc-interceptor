#pragma once

#include <Eigen/Dense>

#include <ilqr/core/types.hpp>
#include <ilqr/cost/cost_concepts.hpp>

namespace guidance
{

/// @brief Running cost ½ (u − u_prev)ᵀ R (u − u_prev), penalizing control deviation rather than
/// control magnitude; zero final cost.
/// @details u_prev is read off the state's tail, not stored separately: this term is meant to be
/// used with a Dynamics model that augments its state with the previously-applied control (e.g.
/// AugmentedDiscreteUAV3DofModel), so the cost has access to u_{k-1} at every stage despite the
/// CostFunction interface only ever seeing (x, u, k).
/// @tparam Dims_T The dimensions the cost operates on
template <typename Dims_T>
class ControlRateCost
{
public:
    using Dims = Dims_T;
    using Scalar = typename Dims_T::Scalar;
    using StateVec = typename Dims_T::StateVec;
    using ControlVec = typename Dims_T::ControlVec;
    using ControlMat = typename Dims_T::ControlMat;

    /// Number of trailing state components holding the previous control.
    static constexpr int control_dim = ControlVec::RowsAtCompileTime;

    /// @brief Builds the term from a control-deviation weight.
    /// @param R Control weighting matrix (expected symmetric positive semi-definite).
    explicit ControlRateCost(const ControlMat& R);

    /// @brief Evaluates the running control-rate cost at (x, u).
    /// @param x The state at timestep k; its tail holds u_{k-1}
    /// @param u The control at this timestep
    /// @param k The timestep index (unused; the penalty is time-invariant)
    /// @return ½ (u − u_prev)ᵀ R (u − u_prev)
    Scalar evaluate(const StateVec& x, const ControlVec& u, int k) const;

    /// @brief Builds the running-cost quadratic expansion at (x, u).
    /// @param x The state at timestep k; its tail holds u_{k-1}
    /// @param u The control at this timestep
    /// @param k The timestep index (unused)
    /// @return Expansion with l, l_x, l_u, l_xx, l_uu and l_ux set
    ilqr::CostTaylorExpansion<Dims_T> quadratize(const StateVec& x, const ControlVec& u,
                                                 int k) const;

    /// @brief Final cost of this term; always zero (this is a running cost).
    /// @param x The final state (unused)
    /// @return Scalar(0)
    Scalar evaluate_final(const StateVec& x) const;

    /// @brief Final-cost expansion of this term; always zero.
    /// @param x The final state (unused)
    /// @return A zero-initialized FinalCostTaylorExpansion
    ilqr::FinalCostTaylorExpansion<Dims_T> quadratize_final(const StateVec& x) const;

private:
    ControlMat R_;
};

}  // namespace guidance

#include "guidance/control_rate_cost_term.inl"
