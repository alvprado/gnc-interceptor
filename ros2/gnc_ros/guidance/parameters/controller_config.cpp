#include "guidance/parameters/controller_config.hpp"

#include <string>
#include <string_view>
#include <utility>

#include "common/parameter_utils.hpp"

namespace gnc_ros
{
namespace
{

constexpr auto prefix = "controller";
constexpr auto vehicle_prefix = "vehicle";

[[nodiscard]] ControllerType controllerTypeFromString(std::string_view name)
{
    if (name == "predictive") return ControllerType::Predictive;
    return ControllerType::ProportionalNavigation;
}

}  // namespace

GuidanceController::GuidanceController(Controller controller) : controller_(std::move(controller))
{
}

Eigen::Vector3d GuidanceController::step(math::CartesianState const& target,
                                         math::CartesianState const& interceptor, double dt)
{
    return std::visit(
        [&](auto& controller) { return controller.step(target, interceptor, dt); }, controller_);
}

ControllerConfig readControllerConfig(rclcpp::Node& node)
{
    ControllerConfig config;
    config.type = controllerTypeFromString(
        readParameter(node, prefix, "type", std::string{"pn"}));

    config.vehicle.mass_kg = readParameter(node, vehicle_prefix, "mass_kg", config.vehicle.mass_kg);
    config.vehicle.rho_kgpm3 =
        readParameter(node, vehicle_prefix, "rho_kgpm3", config.vehicle.rho_kgpm3);
    config.vehicle.frontal_area_m2 =
        readParameter(node, vehicle_prefix, "frontal_area_m2", config.vehicle.frontal_area_m2);
    config.vehicle.drag_coeff =
        readParameter(node, vehicle_prefix, "drag_coeff", config.vehicle.drag_coeff);

    config.min_load_factor = readParameter(node, prefix, "min_load_factor", config.min_load_factor);
    config.max_load_factor = readParameter(node, prefix, "max_load_factor", config.max_load_factor);
    config.max_bank_angle_rad =
        readParameter(node, prefix, "max_bank_angle_rad", config.max_bank_angle_rad);
    config.boost_phase_switch_speed_mps = readParameter(
        node, prefix, "boost_phase_switch_speed_mps", config.boost_phase_switch_speed_mps);
    config.boost_phase_thrust_n =
        readParameter(node, prefix, "boost_phase_thrust_n", config.boost_phase_thrust_n);

    if (config.type == ControllerType::ProportionalNavigation)
    {
        config.navigation_gain =
            readParameter(node, prefix, "navigation_gain", config.navigation_gain);
        return config;
    }

    config.horizon = readParameter(node, prefix, "horizon", config.horizon);
    config.min_horizon = readParameter(node, prefix, "min_horizon", config.min_horizon);
    config.dt = readParameter(node, prefix, "dt", config.dt);
    config.load_factor_rate_weight =
        readParameter(node, prefix, "load_factor_rate_weight", config.load_factor_rate_weight);
    config.bank_rate_weight =
        readParameter(node, prefix, "bank_rate_weight", config.bank_rate_weight);
    config.final_interception_weight = readParameter(
        node, prefix, "final_interception_weight", config.final_interception_weight);
    config.running_interception_weight = readParameter(
        node, prefix, "running_interception_weight", config.running_interception_weight);
    config.d_scale_time_constant_s =
        readParameter(node, prefix, "d_scale_time_constant_s", config.d_scale_time_constant_s);
    return config;
}

GuidanceController makeGuidanceController(ControllerConfig const& config)
{
    if (config.type == ControllerType::Predictive)
    {
        guidance::PredictiveGuidanceControllerConfig predictive_config;
        predictive_config.horizon = config.horizon;
        predictive_config.min_horizon = config.min_horizon;
        predictive_config.dt = config.dt;
        predictive_config.boost_phase_switch_speed_mps = config.boost_phase_switch_speed_mps;
        predictive_config.boost_phase_thrust_n = config.boost_phase_thrust_n;
        predictive_config.model_params = config.vehicle;
        predictive_config.min_load_factor = config.min_load_factor;
        predictive_config.max_load_factor = config.max_load_factor;
        predictive_config.max_bank_angle_rad = config.max_bank_angle_rad;
        predictive_config.control_effort_weight = {config.load_factor_rate_weight,
                                                    config.bank_rate_weight};
        predictive_config.final_interception_weight = config.final_interception_weight;
        predictive_config.running_interception_weight = config.running_interception_weight;
        predictive_config.d_scale_time_constant_s = config.d_scale_time_constant_s;
        return GuidanceController{guidance::PredictiveGuidanceController{predictive_config}};
    }

    guidance::PNControllerConfig pn_config;
    pn_config.navigation_gain = config.navigation_gain;
    pn_config.boost_phase_switch_speed_mps = config.boost_phase_switch_speed_mps;
    pn_config.boost_phase_thrust_n = config.boost_phase_thrust_n;
    pn_config.min_load_factor = config.min_load_factor;
    pn_config.max_load_factor = config.max_load_factor;
    pn_config.max_bank_angle_rad = config.max_bank_angle_rad;
    pn_config.vehicle = config.vehicle;
    return GuidanceController{guidance::PNController{pn_config}};
}

}  // namespace gnc_ros
