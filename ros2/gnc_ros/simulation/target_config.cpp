#include "simulation/target_config.hpp"

#include <string>
#include <string_view>
#include <utility>

#include "common/parameter_utils.hpp"

namespace gnc_ros
{
namespace
{

constexpr auto prefix = "target";

[[nodiscard]] TargetType targetTypeFromString(std::string_view name)
{
    if (name == "constant_velocity") return TargetType::ConstantVelocity;
    if (name == "circle") return TargetType::Circle;
    if (name == "helix") return TargetType::Helix;
    return TargetType::FigureEight;
}

}  // namespace

TargetTrajectory::TargetTrajectory(Maneuver maneuver) : maneuver_(std::move(maneuver)) {}

math::CartesianState TargetTrajectory::evaluateTargetStateAt(double time_s) const
{
    return std::visit([time_s](auto const& maneuver)
                      { return maneuver.evaluateTargetStateAt(time_s); }, maneuver_);
}

TargetConfig readTargetConfig(rclcpp::Node& node)
{
    TargetConfig config;
    config.type =
        targetTypeFromString(readParameter(node, prefix, "type", std::string{"figure_eight"}));

    if (config.type == TargetType::ConstantVelocity)
    {
        config.position_m = readVectorParameter(node, prefix, "position_m", config.position_m);
        config.velocity_mps =
            readVectorParameter(node, prefix, "velocity_mps", config.velocity_mps);
        return config;
    }

    config.center_m = readVectorParameter(node, prefix, "center_m", config.center_m);
    config.normal = readVectorParameter(node, prefix, "normal", config.normal);
    config.reference_direction =
        readVectorParameter(node, prefix, "reference_direction", config.reference_direction);

    if (config.type == TargetType::FigureEight)
    {
        config.length_m = readParameter(node, prefix, "length_m", config.length_m);
        config.width_m = readParameter(node, prefix, "width_m", config.width_m);
        config.angular_rate_rps =
            readParameter(node, prefix, "angular_rate_rps", config.angular_rate_rps);
    }
    else
    {
        config.speed_mps = readParameter(node, prefix, "speed_mps", config.speed_mps);
        config.load_factor = readParameter(node, prefix, "load_factor", config.load_factor);
        if (config.type == TargetType::Helix)
        {
            config.climb_angle_rad =
                readParameter(node, prefix, "climb_angle_rad", config.climb_angle_rad);
        }
    }
    return config;
}

TargetTrajectory makeTargetTrajectory(TargetConfig const& config)
{
    switch (config.type)
    {
        case TargetType::ConstantVelocity:
            return TargetTrajectory{
                target::ConstantVelocity{config.position_m, config.velocity_mps}};
        case TargetType::Circle:
            return TargetTrajectory{target::Circle{config.center_m, config.speed_mps,
                                                   config.load_factor, config.normal,
                                                   config.reference_direction}};
        case TargetType::Helix:
            return TargetTrajectory{target::Helix{config.center_m, config.speed_mps,
                                                  config.load_factor, config.climb_angle_rad,
                                                  config.normal, config.reference_direction}};
        case TargetType::FigureEight:
        default:
            return TargetTrajectory{target::FigureEight{config.center_m, config.length_m,
                                                        config.width_m, config.angular_rate_rps,
                                                        config.normal, config.reference_direction}};
    }
}

}  // namespace gnc_ros
