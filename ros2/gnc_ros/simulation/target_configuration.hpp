#pragma once

#include <variant>

#include "target/maneuvers.hpp"

namespace rclcpp
{
class Node;
}

namespace gnc_ros
{

enum class TargetType
{
    ConstantVelocity,
    Circle,
    FigureEight,
    Helix,
};

/// @brief Target construction parameters; only fields used by the selected type are read.
struct TargetConfig
{
    TargetType type{TargetType::FigureEight};
    Eigen::Vector3d position_m{3000.0, 500.0, 1500.0};
    Eigen::Vector3d velocity_mps{30.0, 0.0, 0.0};
    Eigen::Vector3d center_m{3000.0, 500.0, 1500.0};
    Eigen::Vector3d normal{1.0, 0.0, 1.0};
    Eigen::Vector3d reference_direction{0.0, -500.0, 0.0};
    double speed_mps{50.0};
    double load_factor{1.0};
    double length_m{1000.0};
    double width_m{500.0};
    double angular_rate_rps{0.1};
    double climb_angle_rad{0.1};
};

/// @brief A trajectory selected at runtime, with the same evaluation API as the core maneuvers.
class TargetTrajectory
{
public:
    using Maneuver =
        std::variant<target::ConstantVelocity, target::Circle, target::FigureEight, target::Helix>;

    explicit TargetTrajectory(Maneuver maneuver);

    [[nodiscard]] math::CartesianState evaluateTargetStateAt(double time_s) const;

private:
    Maneuver maneuver_;
};

static_assert(target::TargetTrajectory<TargetTrajectory>);

/// @brief Declare and read the selected target's startup-only ROS parameters.
[[nodiscard]] TargetConfig readTargetConfig(rclcpp::Node& node);

/// @brief Construct the selected core maneuver from its configuration.
[[nodiscard]] TargetTrajectory makeTargetTrajectory(TargetConfig const& config);

}  // namespace gnc_ros
