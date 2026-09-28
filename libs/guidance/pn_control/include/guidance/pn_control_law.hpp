#pragma once

#include <Eigen/Dense>

#include "math/cartesian_state.hpp"

namespace guidance
{

/// @brief Proportional Navigation Policy
class ProportionalNavigationControlLaw
{
public:
    /// @brief Construct a PN policy from the navigation gain
    explicit ProportionalNavigationControlLaw(double navigation_gain);

    /// @brief Step the PN policy.
    /// @param[in] target The target's Cartesian state.
    /// @param[in] interceptor The interceptor's Cartesian state.
    /// @returns The commanded acceleration, in the inertial frame.
    [[nodiscard]] Eigen::Vector3d step(math::CartesianState const& target,
                                       math::CartesianState const& interceptor) const noexcept;

private:
    double navigation_gain_{3.0};
};

}  // namespace guidance
