#pragma once

#include <Eigen/Geometry>

#include <cmath>
#include <numbers>

namespace math
{

/// @brief Wrap an angle into (-pi, pi].
/// @param[in] angle_rad The angle to wrap, in radians.
/// @returns The equivalent angle in (-pi, pi].
[[nodiscard]] inline double wrapToPi(double angle_rad) noexcept
{
    return std::remainder(angle_rad, 2.0 * std::numbers::pi);
}

/// @brief Body attitude from heading, flight-path angle and bank, assuming
/// zero sideslip and angle of attack: the body's forward (x) axis is aligned
/// with the velocity direction (heading, flight-path angle), then rolled
/// about that axis by bank.
/// @details Standard 3-2-1 (yaw-pitch-roll) composition, R = Rz(heading) *
/// Ry(-flight_path_angle) * Rx(bank). The pitch term is negated because this
/// project's frame is z-up.
/// @param[in] heading_rad Yaw about the inertial +z axis.
/// @param[in] flight_path_angle_rad Climb angle above the horizontal; positive climbs.
/// @param[in] bank_rad Roll about the body's forward (velocity) axis.
/// @returns The body-to-inertial attitude as a unit quaternion.
[[nodiscard]] inline Eigen::Quaterniond attitudeFromHeadingPitchBank(double heading_rad,
                                                                     double flight_path_angle_rad,
                                                                     double bank_rad) noexcept
{
    Eigen::Quaterniond const yaw{Eigen::AngleAxisd(heading_rad, Eigen::Vector3d::UnitZ())};
    Eigen::Quaterniond const pitch{
        Eigen::AngleAxisd(-flight_path_angle_rad, Eigen::Vector3d::UnitY())};
    Eigen::Quaterniond const roll{Eigen::AngleAxisd(bank_rad, Eigen::Vector3d::UnitX())};
    return yaw * pitch * roll;
}

/// @brief Heading, flight-path angle and bank, as built by
/// attitudeFromHeadingPitchBank().
struct EulerAngles
{
    double heading_rad{0.0};
    double flight_path_angle_rad{0.0};
    double bank_rad{0.0};
};

/// @brief Exact inverse of attitudeFromHeadingPitchBank(), for
/// flight_path_angle_rad in [-pi/2, pi/2] (this project's convention).
/// @param[in] attitude A body-to-inertial attitude built by
/// attitudeFromHeadingPitchBank().
/// @returns The heading, flight-path angle and bank that produced it.
[[nodiscard]] inline EulerAngles eulerAnglesFromAttitude(Eigen::Quaterniond const& attitude) noexcept
{
    Eigen::Matrix3d const rotation = attitude.toRotationMatrix();

    EulerAngles result;
    result.heading_rad = std::atan2(rotation(1, 0), rotation(0, 0));
    result.flight_path_angle_rad = std::atan2(
        rotation(2, 0), std::sqrt(rotation(0, 0) * rotation(0, 0) + rotation(1, 0) * rotation(1, 0)));
    result.bank_rad = std::atan2(rotation(2, 1), rotation(2, 2));
    return result;
}

}  // namespace math
