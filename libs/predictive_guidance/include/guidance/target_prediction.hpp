#pragma once

#include <cstddef>
#include <Eigen/Dense>
#include <ilqr/core/types.hpp>
#include <vector>

#include "math/cartesian_state.hpp"

namespace guidance
{

/// @brief Predict the target's future positions assuming constant velocity.
/// @param[in] initial_state The target's Cartesian state at t=0.
/// @param[in] horizon Number of predicted positions to return.
/// @param[in] dt Time step between predictions.
/// @returns Predicted positions at t = 0, dt, 2*dt, ..., (horizon-1)*dt.
[[nodiscard]] inline ilqr::AlignedVec<Eigen::Vector3d> constantVelocityTargetPositionPrediction(
    math::CartesianState const& initial_state, std::size_t horizon, double dt) noexcept
{
    ilqr::AlignedVec<Eigen::Vector3d> prediction(horizon);
    for (std::size_t i{0}; i < horizon; ++i)
    {
        prediction[i] =
            initial_state.position_m + static_cast<double>(i) * dt * initial_state.velocity_mps;
    }
    return prediction;
}

}  // namespace guidance
