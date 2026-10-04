#include "simulation/parameters/interceptor_config.hpp"

#include "common/parameter_utils.hpp"

namespace gnc_ros
{
namespace
{

constexpr auto prefix = "interceptor";

}  // namespace

InterceptorConfig readInterceptorConfig(rclcpp::Node& node)
{
    InterceptorConfig config;
    config.params.mass_kg = readParameter(node, prefix, "mass_kg", config.params.mass_kg);
    config.params.rho_kgpm3 = readParameter(node, prefix, "rho_kgpm3", config.params.rho_kgpm3);
    config.params.frontal_area_m2 =
        readParameter(node, prefix, "frontal_area_m2", config.params.frontal_area_m2);
    config.params.drag_coeff =
        readParameter(node, prefix, "drag_coeff", config.params.drag_coeff);

    config.limits.min_thrust_n =
        readParameter(node, prefix, "min_thrust_n", config.limits.min_thrust_n);
    config.limits.max_thrust_n =
        readParameter(node, prefix, "max_thrust_n", config.limits.max_thrust_n);
    config.limits.min_load_factor =
        readParameter(node, prefix, "min_load_factor", config.limits.min_load_factor);
    config.limits.max_load_factor =
        readParameter(node, prefix, "max_load_factor", config.limits.max_load_factor);
    config.limits.max_bank_angle_rad =
        readParameter(node, prefix, "max_bank_angle_rad", config.limits.max_bank_angle_rad);
    config.limits.min_speed_mps =
        readParameter(node, prefix, "min_speed_mps", config.limits.min_speed_mps);
    config.limits.max_speed_mps =
        readParameter(node, prefix, "max_speed_mps", config.limits.max_speed_mps);

    return config;
}

simulation::UAV3DofModel makeInterceptorModel(InterceptorConfig const& config)
{
    return simulation::UAV3DofModel{config.params, config.limits};
}

}  // namespace gnc_ros
