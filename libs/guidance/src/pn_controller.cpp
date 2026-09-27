#include "guidance/pn_controller.hpp"

#include "guidance/pn_control_law.hpp"
#include "guidance/thrust_control_law.hpp"
#include "guidance/transverse_control_allocation.hpp"

namespace guidance
{

PNController::PNController(PNControllerConfig const& config) noexcept
    : config_(config),
      pn_control_law_(ProportionalNavigationControlLaw{config.navigation_gain}),
      thrust_control_law_(ThrustControlLaw(ThrustControlConfig{config.vehicle})),
      transverse_control_allocation_(TransverseControlAllocation{TransverseControlAllocationConfig{
          config.min_load_factor, config.max_load_factor, config.max_bank_angle_rad}})
{
}

Eigen::Vector3d PNController::step(math::CartesianState const& target,
                                   math::CartesianState const& interceptor, double) const noexcept
{
    if (interceptor.velocity_mps.norm() < config_.boost_phase_switch_speed_mps)
    {
        return Eigen::Vector3d{config_.boost_phase_thrust_n, 1.0, 0.0};
    }

    double const thrust = thrust_control_law_.step(interceptor);
    Eigen::Vector3d const transverse_acceleration_cmd = pn_control_law_.step(target, interceptor);
    auto const transverse_allocation_output =
        transverse_control_allocation_.step(interceptor, transverse_acceleration_cmd);

    return Eigen::Vector3d{thrust, transverse_allocation_output.load_factor,
                           transverse_allocation_output.bank_angle_rad};
}

}  // namespace guidance
