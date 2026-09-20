#include "guidance/pn_controller.hpp"

#include "guidance/pn_control_law.hpp"
#include "guidance/thrust_control_law.hpp"
#include "guidance/transverse_control_allocation.hpp"

namespace guidance
{

PNController::PNController(PNControllerConfig const& config) noexcept
    : pn_control_law_(ProportionalNavigationControlLaw{config.navigation_gain}),
      thrust_control_law_(ThrustControlLaw{config.thrust_control_config}),
      transverse_control_allocation_(
          TransverseControlAllocation{config.transverse_control_allocation_config})
{
}

Eigen::Vector3d PNController::step(math::CartesianState const& target,
                                   math::CartesianState const& interceptor, double) const noexcept
{
    double const thrust = thrust_control_law_.step(interceptor);

    if (interceptor.velocity_mps.norm() < thrust_control_law_.switchSpeedMps())
    {
        return Eigen::Vector3d{thrust, 1.0, 0.0};
    }

    Eigen::Vector3d const transverse_acceleration_cmd = pn_control_law_.step(target, interceptor);
    auto const transverse_allocation_output =
        transverse_control_allocation_.step(interceptor, transverse_acceleration_cmd);

    return Eigen::Vector3d{thrust, transverse_allocation_output.load_factor,
                           transverse_allocation_output.bank_angle_rad};
}

}  // namespace guidance
