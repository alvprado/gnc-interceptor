#pragma once

#include <Eigen/Dense>
#include <ilqr/cost/cost_concepts.hpp>

#include "guidance/types.hpp"

namespace guidance
{

class SoftminCost
{
public:
    /// Aliases
    using Dims = guidance::Dims;
    using Scalar = typename Dims::Scalar;
    using StateVec = typename Dims::StateVec;
    using ControlVec = typename Dims::ControlVec;
    using ControlMat = typename Dims::ControlMat;

    /// @brief Ctor
    /// @param[in] config The softmin tuning (d_scale, min_q, beta).
    /// @param[in] last_target_position The target position to soft-min the distance to.
    /// @param[in] softmin_weight Weight of this cost
    SoftminCost(SoftminConfig const& config, Eigen::Vector3d const& last_target_position,
                double softmin_weight) noexcept
        : config_(config),
          last_target_position_(last_target_position),
          softmin_weight_(softmin_weight)
    {
    }

    /// @brief Zero running costs.
    [[nodiscard]] Scalar evaluate(const StateVec&, const ControlVec&, int) const
    {
        return Scalar(0);
    }

    /// @brief Softmin cost J_softmin
    [[nodiscard]] Scalar evaluate_final(const StateVec& x) const
    {
        return softmin_weight_ *
               (config_.min_q -
                (1 / config_.beta) *
                    std::log(x[6] + std::exp(-config_.beta *
                                             (scaledSquaredDistanceToTarget(x) - config_.min_q))));
    }

    /// @brief Running-cost quadratic expansion (the final-cost expansion scaled by running_weight).
    [[nodiscard]] ilqr::CostTaylorExpansion<Dims> quadratize(const StateVec&, const ControlVec&,
                                                             int) const
    {
        return ilqr::CostTaylorExpansion<Dims>{};
    }

    /// @brief Final-cost quadratic expansion with the exact (indefinite) gradient/Hessian.
    [[nodiscard]] ilqr::FinalCostTaylorExpansion<Dims> quadratize_final(const StateVec& x) const
    {
        /// Softmin helpers
        Scalar const s =
            std::exp(-config_.beta * (scaledSquaredDistanceToTarget(x) - config_.min_q));
        Scalar const D = x[6] + s;
        Scalar const weight = s / D;

        /// Relative vector
        Eigen::Vector3d const r = last_target_position_ - x.head<3>();

        /// Gradient of q w.r.t. nominal state x
        Eigen::Vector<Scalar, 6> q_x = Eigen::Vector<Scalar, 6>::Zero();
        q_x.template head<3>() = -2 * r / (config_.d_scale * config_.d_scale);

        /// Hessian of q w.r.t. nominal state x
        Eigen::Matrix<Scalar, 6, 6> q_xx = Eigen::Matrix<Scalar, 6, 6>::Zero();
        q_xx(0, 0) = 2 / (config_.d_scale * config_.d_scale);
        q_xx(1, 1) = q_xx(0, 0);
        q_xx(2, 2) = q_xx(0, 0);

        /// Gradient and Hessian of full state
        ilqr::FinalCostTaylorExpansion<Dims> result{};
        result.lf = config_.min_q - (1.0 / config_.beta) * std::log(D);
        result.lf_x(Eigen::seqN(0, 6)) = weight * q_x;
        result.lf_x[6] = -1.0 / (config_.beta * D);
        result.lf_xx(Eigen::seqN(0, 6), Eigen::seqN(0, 6)) =
            weight * q_xx - config_.beta * weight * (1 - weight) * q_x * q_x.transpose();
        result.lf_xx(Eigen::seqN(6, 1), Eigen::seqN(0, 6)) = -(weight / D) * q_x.transpose();
        result.lf_xx(Eigen::seqN(0, 6), Eigen::seqN(6, 1)) = -(weight / D) * q_x;
        result.lf_xx(6, 6) = 1.0 / (config_.beta * D * D);

        result.lf *= softmin_weight_;
        result.lf_x *= softmin_weight_;
        result.lf_xx *= softmin_weight_;

        return result;
    }

private:
    [[nodiscard]] Scalar scaledSquaredDistanceToTarget(const StateVec& x) const
    {
        return ((last_target_position_.x() - x[0]) * (last_target_position_.x() - x[0]) +
                (last_target_position_.y() - x[1]) * (last_target_position_.y() - x[1]) +
                (last_target_position_.z() - x[2]) * (last_target_position_.z() - x[2])) /
               (config_.d_scale * config_.d_scale);
    }

    SoftminConfig config_;
    Eigen::Vector3d last_target_position_;
    double softmin_weight_;
};
}  // namespace guidance
