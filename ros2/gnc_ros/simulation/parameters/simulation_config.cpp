#include "simulation/parameters/simulation_config.hpp"

#include <chrono>
#include <cmath>
#include <optional>
#include <string>
#include <string_view>

#include "common/parameter_utils.hpp"

namespace gnc_ros
{
namespace
{

constexpr auto prefix = "simulation";

[[nodiscard]] std::optional<IntegrationMethod> integrationMethodFromString(
    std::string_view name) noexcept
{
    if (name == "rk4") return IntegrationMethod::RK4;
    if (name == "heun") return IntegrationMethod::Heun;
    if (name == "euler") return IntegrationMethod::Euler;
    return std::nullopt;
}

}  // namespace

SimConfig readSimulationConfig(rclcpp::Node& node)
{
    SimConfig config;
    auto const dt_s = readParameter(node, prefix, "dt_s", config.dt_s);
    constexpr double max_dt_s =
        std::chrono::duration<double>{std::chrono::nanoseconds::max()}.count() - 1.0;
    if (!std::isfinite(dt_s) || dt_s < 1.0e-9 || dt_s > max_dt_s)
    {
        RCLCPP_WARN(node.get_logger(), "Invalid simulation.dt_s (%g); using %g s.",
                    dt_s, config.dt_s);
    }
    else
    {
        config.dt_s = dt_s;
    }
    auto const method_name =
        readParameter(node, prefix, "integration_method", std::string{"rk4"});
    auto const method = integrationMethodFromString(method_name);
    if (!method)
    {
        RCLCPP_WARN(node.get_logger(),
                    "Unknown simulation.integration_method '%s'; using rk4.",
                    method_name.c_str());
        return config;
    }
    switch (*method)
    {
        case IntegrationMethod::RK4:
            config.integrator = math::RK4Step{};
            break;
        case IntegrationMethod::Heun:
            config.integrator = math::HeunStep{};
            break;
        case IntegrationMethod::Euler:
            config.integrator = math::EulerStep{};
            break;
    }
    return config;
}

}  // namespace gnc_ros
