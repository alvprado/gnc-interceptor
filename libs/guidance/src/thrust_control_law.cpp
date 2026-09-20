#include "guidance/thrust_control_law.hpp"

#include <algorithm>
#include <cmath>

#include "math/constants.hpp"

namespace guidance
{

ThrustControlLaw::ThrustControlLaw(ThrustControlConfig const& config) noexcept : config_(config)
{
}

double ThrustControlLaw::step(math::CartesianState const& interceptor) const noexcept
{
    double const speed = interceptor.velocity_mps.norm();

    if (speed < config_.switch_speed_mps)
    {
        return config_.max_thrust_n;
    }

    double const flight_path_angle =
        std::asin(std::clamp(interceptor.velocity_mps.z() / speed, -1.0, 1.0));

    auto const& vehicle = config_.vehicle;
    double const drag_force =
        0.5 * vehicle.rho_kgpm3 * vehicle.frontal_area_m2 * vehicle.drag_coeff * speed * speed;

    return drag_force + math::gravity_mps2 * std::sin(flight_path_angle) * vehicle.mass_kg;
}

double ThrustControlLaw::switchSpeedMps() const noexcept
{
    return config_.switch_speed_mps;
}

}  // namespace guidance
