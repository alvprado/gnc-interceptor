#include "target/maneuvers.hpp"

#include <cmath>

namespace target
{

ConstantVelocity::ConstantVelocity(Eigen::Vector3d const& position_m,
                                   Eigen::Vector3d const& velocity_mps)
    : initial_position_m_(position_m), velocity_mps_(velocity_mps)
{
}

TargetState ConstantVelocity::evaluateTargetStateAt(double time_s) const noexcept
{
    return TargetState{initial_position_m_ + time_s * velocity_mps_, velocity_mps_,
                       Eigen::Vector3d::Zero()};
}

Circle::Circle(Eigen::Vector3d const& center_m, double speed_mps, double load_factor,
               Eigen::Vector3d const& normal, Eigen::Vector3d const& reference_direction)
    : center_m_(center_m),
      radius_m_(detail::turnRadius(speed_mps, load_factor)),
      angular_rate_rps_(detail::angularRate(speed_mps, load_factor)),
      basis_(detail::planeBasis(normal, reference_direction))
{
}

TargetState Circle::evaluateTargetStateAt(double time_s) const noexcept
{
    double const theta = angular_rate_rps_ * time_s;
    Eigen::Vector3d const radial =
        radius_m_ * (std::cos(theta) * basis_.u + std::sin(theta) * basis_.w);
    Eigen::Vector3d const tangential =
        radius_m_ * angular_rate_rps_ * (-std::sin(theta) * basis_.u + std::cos(theta) * basis_.w);

    TargetState state;
    state.position_m = center_m_ + radial;
    state.velocity_mps = tangential;
    state.acceleration_mps2 = -angular_rate_rps_ * angular_rate_rps_ * radial;
    return state;
}

FigureEight::FigureEight(Eigen::Vector3d const& center_m, double length_m, double width_m,
                         double angular_rate_rps, Eigen::Vector3d const& normal,
                         Eigen::Vector3d const& reference_direction)
    : center_m_(center_m),
      half_length_m_(0.5 * length_m),
      half_width_m_(0.5 * width_m),
      angular_rate_rps_(angular_rate_rps),
      basis_(detail::planeBasis(normal, reference_direction))
{
}

TargetState FigureEight::evaluateTargetStateAt(double time_s) const noexcept
{
    double const theta = angular_rate_rps_ * time_s;
    double const sin_t = std::sin(theta), cos_t = std::cos(theta);
    double const sin_2t = std::sin(2.0 * theta), cos_2t = std::cos(2.0 * theta);

    double const x = half_length_m_ * sin_t;
    double const y = half_width_m_ * sin_2t;
    double const dx = half_length_m_ * cos_t;
    double const dy = 2.0 * half_width_m_ * cos_2t;
    double const ddx = -half_length_m_ * sin_t;
    double const ddy = -4.0 * half_width_m_ * sin_2t;

    TargetState state;
    state.position_m = center_m_ + x * basis_.u + y * basis_.w;
    state.velocity_mps = angular_rate_rps_ * (dx * basis_.u + dy * basis_.w);
    state.acceleration_mps2 =
        angular_rate_rps_ * angular_rate_rps_ * (ddx * basis_.u + ddy * basis_.w);
    return state;
}

Helix::Helix(Eigen::Vector3d const& center_m, double speed_mps, double load_factor,
             double climb_angle_rad, Eigen::Vector3d const& normal,
             Eigen::Vector3d const& reference_direction)
    : center_m_(center_m),
      radius_m_(detail::turnRadius(speed_mps * std::cos(climb_angle_rad), load_factor)),
      angular_rate_rps_(detail::angularRate(speed_mps * std::cos(climb_angle_rad), load_factor)),
      climb_rate_mps_(speed_mps * std::sin(climb_angle_rad)),
      basis_(detail::planeBasis(normal, reference_direction))
{
}

TargetState Helix::evaluateTargetStateAt(double time_s) const noexcept
{
    double const theta = angular_rate_rps_ * time_s;
    Eigen::Vector3d const radial =
        radius_m_ * (std::cos(theta) * basis_.u + std::sin(theta) * basis_.w);
    Eigen::Vector3d const tangential =
        radius_m_ * angular_rate_rps_ * (-std::sin(theta) * basis_.u + std::cos(theta) * basis_.w);

    TargetState state;
    state.position_m = center_m_ + radial + climb_rate_mps_ * time_s * basis_.n_hat;
    state.velocity_mps = tangential + climb_rate_mps_ * basis_.n_hat;
    state.acceleration_mps2 = -angular_rate_rps_ * angular_rate_rps_ * radial;
    return state;
}

}  // namespace target
