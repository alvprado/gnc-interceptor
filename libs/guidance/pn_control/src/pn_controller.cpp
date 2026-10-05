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
    double const speed = interceptor.velocity_mps.norm();
    if (speed < config_.boost_phase_switch_speed_mps)
    {
        double const load =
            speed > 1.0e-6 ? interceptor.velocity_mps.head<2>().norm() / speed : 1.0;
        return Eigen::Vector3d{config_.boost_phase_thrust_n, load, 0.0};
    }

    double const thrust = thrust_control_law_.step(interceptor);
    Eigen::Vector3d const transverse_acceleration_cmd = pn_control_law_.step(target, interceptor);
    auto const transverse_allocation_output =
        transverse_control_allocation_.step(interceptor, transverse_acceleration_cmd);

    return Eigen::Vector3d{thrust, transverse_allocation_output.load_factor,
                           transverse_allocation_output.bank_angle_rad};
}

}  // namespace guidance
