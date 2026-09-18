#pragma once

#include <Eigen/Dense>

#include "target/concepts.hpp"
#include "target/geometry.hpp"

namespace target
{

/// @brief Straight-line, constant-velocity target trajectory.
class ConstantVelocity
{
public:
    /// @brief Construct from the initial position and the (constant) velocity.
    /// @param[in] position_m Position at t = 0.
    /// @param[in] velocity_mps Velocity, held constant for all t.
    ConstantVelocity(Eigen::Vector3d const& position_m, Eigen::Vector3d const& velocity_mps);

    /// @brief Evaluate the target state at the given time.
    /// @param[in] time_s Time since t = 0.
    /// @returns The position, velocity and (zero) acceleration at time_s.
    [[nodiscard]] TargetState evaluateTargetStateAt(double time_s) const noexcept;

private:
    Eigen::Vector3d initial_position_m_;
    Eigen::Vector3d velocity_mps_;
};

/// @brief Constant-speed, constant-load-factor circular target trajectory.
/// @details Parameterized the way a maneuver is commanded operationally - a
/// speed and a turn load factor - rather than by radius and turn rate
/// directly; radius and angular rate follow from R = v^2 / (n g) and
/// omega = n g / v.
class Circle
{
public:
    /// @brief Construct a circular trajectory in an arbitrary 3D plane.
    /// @param[in] center_m Center of the circle.
    /// @param[in] speed_mps Speed along the circle, held constant.
    /// @param[in] load_factor Lateral load factor of the turn; sign sets the
    /// turn direction. Must be nonzero.
    /// @param[in] normal Normal to the circle's plane (need not be unit length).
    /// @param[in] reference_direction A vector, not parallel to normal, that
    /// fixes the target's position at t = 0.
    Circle(Eigen::Vector3d const& center_m, double speed_mps, double load_factor,
           Eigen::Vector3d const& normal, Eigen::Vector3d const& reference_direction);

    /// @brief Evaluate the target state at the given time.
    /// @param[in] time_s Time since t = 0.
    /// @returns The position, velocity and centripetal acceleration at time_s.
    [[nodiscard]] TargetState evaluateTargetStateAt(double time_s) const noexcept;

private:
    Eigen::Vector3d center_m_;
    double radius_m_;
    double angular_rate_rps_;
    detail::PlaneBasis basis_;
};

/// @brief Figure-eight (Gerono lemniscate) target trajectory.
/// @details Traces x = A sin(theta), y = B sin(theta) cos(theta) in the
/// (u, w) plane, i.e. a single lobe every half period - the "figure eight"
/// shape a Lissajous curve with a 2:1 frequency ratio traces in-plane.
class FigureEight
{
public:
    /// @brief Construct a figure-eight trajectory in an arbitrary 3D plane.
    /// @param[in] center_m Center of the figure eight.
    /// @param[in] length_m Extent along the u axis (tip to tip).
    /// @param[in] width_m Extent along the w axis (tip to tip).
    /// @param[in] angular_rate_rps Rate at which the parameter theta advances.
    /// @param[in] normal Normal to the figure-eight's plane.
    /// @param[in] reference_direction A vector, not parallel to normal, fixing
    /// the u axis (the long axis, through both lobes).
    FigureEight(Eigen::Vector3d const& center_m, double length_m, double width_m,
                double angular_rate_rps, Eigen::Vector3d const& normal,
                Eigen::Vector3d const& reference_direction);

    /// @brief Evaluate the target state at the given time.
    /// @param[in] time_s Time since t = 0.
    /// @returns The position, velocity and acceleration at time_s.
    [[nodiscard]] TargetState evaluateTargetStateAt(double time_s) const noexcept;

private:
    Eigen::Vector3d center_m_;
    double half_length_m_;
    double half_width_m_;
    double angular_rate_rps_;
    detail::PlaneBasis basis_;
};

/// @brief Helical (constant-radius, constant climb-rate) 3D target trajectory.
/// @details A circle in the (u, w) plane advanced along the plane normal -
/// a "3D spiral" in the sense of a circle that also moves forward, not an
/// expanding-radius spiral. Parameterized by the total speed, the load
/// factor of the horizontal turn, and the climb angle, matching how a
/// climbing/descending turn is commanded operationally.
class Helix
{
public:
    /// @brief Construct a helical trajectory.
    /// @param[in] center_m Center of the helix at t = 0.
    /// @param[in] speed_mps Total speed along the trajectory, held constant.
    /// @param[in] load_factor Lateral load factor of the horizontal turn
    /// component; sign sets the turn direction. Must be nonzero.
    /// @param[in] climb_angle_rad Flight-path angle of the helix; positive
    /// climbs, negative descends.
    /// @param[in] normal Normal to the circular component's plane, i.e. the
    /// helix axis (need not be unit length).
    /// @param[in] reference_direction A vector, not parallel to normal, that
    /// fixes the target's position at t = 0.
    Helix(Eigen::Vector3d const& center_m, double speed_mps, double load_factor,
          double climb_angle_rad, Eigen::Vector3d const& normal,
          Eigen::Vector3d const& reference_direction);

    /// @brief Evaluate the target state at the given time.
    /// @param[in] time_s Time since t = 0.
    /// @returns The position, velocity and acceleration at time_s.
    [[nodiscard]] TargetState evaluateTargetStateAt(double time_s) const noexcept;

private:
    Eigen::Vector3d center_m_;
    double radius_m_;
    double angular_rate_rps_;
    double climb_rate_mps_;
    detail::PlaneBasis basis_;
};

static_assert(TargetTrajectory<ConstantVelocity>);
static_assert(TargetTrajectory<Circle>);
static_assert(TargetTrajectory<FigureEight>);
static_assert(TargetTrajectory<Helix>);

}  // namespace target
