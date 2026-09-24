#pragma once

#include <cassert>

#include "guidance/control_rate_cost_term.hpp"

namespace guidance
{

template <typename Dims_T>
ControlRateCost<Dims_T>::ControlRateCost(const ControlMat& R) : R_(R)
{
    assert(R_.isApprox(R_.transpose()) && "R must be symmetric");
}

template <typename Dims_T>
auto ControlRateCost<Dims_T>::evaluate(const StateVec& x, const ControlVec& u, int) const -> Scalar
{
    const ControlVec du = u - x.template tail<control_dim>();
    return Scalar(0.5) * du.dot(R_ * du);
}

template <typename Dims_T>
auto ControlRateCost<Dims_T>::quadratize(const StateVec& x, const ControlVec& u, int) const
    -> ilqr::CostTaylorExpansion<Dims_T>
{
    auto cost_taylor_expansion = ilqr::CostTaylorExpansion<Dims_T>{};

    const ControlVec du = u - x.template tail<control_dim>();
    const ControlVec R_du = R_ * du;

    cost_taylor_expansion.l = Scalar(0.5) * du.dot(R_du);
    cost_taylor_expansion.l_u = R_du;
    cost_taylor_expansion.l_uu = R_;

    // du = u - x.tail(u_prev): only the state's previous-control tail enters the cost, so every
    // state-facing block is zero except where it overlaps that tail.
    cost_taylor_expansion.l_x.template tail<control_dim>() = -R_du;
    cost_taylor_expansion.l_xx.template bottomRightCorner<control_dim, control_dim>() = R_;
    cost_taylor_expansion.l_ux.template rightCols<control_dim>() = -R_;

    return cost_taylor_expansion;
}

template <typename Dims_T>
auto ControlRateCost<Dims_T>::evaluate_final(const StateVec&) const -> Scalar
{
    return Scalar(0);
}

template <typename Dims_T>
auto ControlRateCost<Dims_T>::quadratize_final(const StateVec&) const
    -> ilqr::FinalCostTaylorExpansion<Dims_T>
{
    return ilqr::FinalCostTaylorExpansion<Dims_T>{};
}

}  // namespace guidance
