#include "guidance/transverse_control_allocation.hpp"

#include <algorithm>
#include <cmath>

#include "math/constants.hpp"

namespace guidance
{
namespace
{

/// @brief Floor on interceptor speed (m/s) applied before normalizing.
inline constexpr double min_speed_mps{1.0e-6};

/// @brief Floor on horizontal speed direction (unitless, cos(gamma))
/// applied before it is used as a division denominator.
inline constexpr double min_horizontal_component{1.0e-6};

}  // namespace

TransverseControlAllocation::TransverseControlAllocation(
    TransverseControlAllocationConfig const& config) noexcept
    : config_(config)
{
}

TransverseControlAllocationOutput TransverseControlAllocation::step(
    math::CartesianState const& interceptor,
    Eigen::Vector3d const& transverse_acceleration_cmd) const noexcept
{
    double const speed_safe = std::max(interceptor.velocity_mps.norm(), min_speed_mps);
    Eigen::Vector3d const u = interceptor.velocity_mps / speed_safe;

    double const cos_gamma = std::hypot(u.x(), u.y());
    double const cos_gamma_safe = std::max(cos_gamma, min_horizontal_component);

    Eigen::Vector3d const e_gamma{-u.z() * u.x() / cos_gamma_safe, -u.z() * u.y() / cos_gamma_safe,
                                  cos_gamma_safe};
    Eigen::Vector3d const e_psi{-u.y() / cos_gamma_safe, u.x() / cos_gamma_safe, 0.0};

    double const a_gamma = transverse_acceleration_cmd.dot(e_gamma);
    double const a_psi = transverse_acceleration_cmd.dot(e_psi);
    double const gamma_term = a_gamma + math::gravity_mps2 * cos_gamma;

    double const load_factor =
        std::sqrt(gamma_term * gamma_term + a_psi * a_psi) / math::gravity_mps2;
    double const bank_angle_rad = std::atan2(a_psi, gamma_term);

    return TransverseControlAllocationOutput{
        std::clamp(load_factor, config_.min_load_factor, config_.max_load_factor),
        std::clamp(bank_angle_rad, -config_.max_bank_angle_rad, config_.max_bank_angle_rad)};
}

}  // namespace guidance
