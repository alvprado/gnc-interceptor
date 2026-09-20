#include "guidance/pn_control_law.hpp"

#include <algorithm>

namespace guidance
{
namespace
{

/// @brief Floor on range (m) applied to the closing-rate and PN denominators.
inline constexpr double min_range_m{1.0e-3};

/// @brief Floor on interceptor speed (m/s) applied before normalizing its
/// velocity direction.
inline constexpr double min_speed_mps{1.0e-6};

}  // namespace

ProportionalNavigationControlLaw::ProportionalNavigationControlLaw(double navigation_gain)
    : navigation_gain_(navigation_gain)
{
}

Eigen::Vector3d ProportionalNavigationControlLaw::step(
    math::CartesianState const& target, math::CartesianState const& interceptor) const noexcept
{
    // Relative position vector
    Eigen::Vector3d const r = target.position_m - interceptor.position_m;

    // Relative velocity vector
    Eigen::Vector3d const v_r = target.velocity_mps - interceptor.velocity_mps;

    // Range
    double const range = r.norm();
    double const range_safe = std::max(range, min_range_m);
    // Closing speed as the negative of the range rate
    double const closing_speed = -r.dot(v_r) / range_safe;

    // LOS angular rate
    Eigen::Vector3d const omega_los = (r.cross(v_r)) / (range_safe * range_safe);

    // Interceptor velocity direction
    double const interceptor_speed_safe = std::max(interceptor.velocity_mps.norm(), min_speed_mps);

    // PN acceleration
    return navigation_gain_ * closing_speed * omega_los.cross(interceptor.velocity_mps) /
           interceptor_speed_safe;
}

}  // namespace guidance
