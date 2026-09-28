#pragma once

#include <Eigen/Dense>

namespace target
{

namespace detail
{

/// @brief An orthonormal (u, w, n_hat) frame spanning a plane, as built by
/// planeBasis.
struct PlaneBasis
{
    Eigen::Vector3d u;
    Eigen::Vector3d w;
    Eigen::Vector3d n_hat;
};

/// @brief Build an orthonormal (u, w) basis spanning the plane normal to n.
/// @details u is the component of `reference` orthogonal to n; w completes a
/// right-handed (u, w, n) frame, so motion parameterized as
/// u*cos(theta) + w*sin(theta) advances counterclockwise as seen from +n.
/// @param[in] normal The plane normal (need not be unit length).
/// @param[in] reference A vector not parallel to normal, fixing the t = 0
/// direction.
/// @returns The orthonormal (u, w, n_hat) frame.
[[nodiscard]] PlaneBasis planeBasis(Eigen::Vector3d const& normal,
                                    Eigen::Vector3d const& reference);

/// @brief Turn radius for a coordinated turn at the given speed and load
/// factor: R = v^2 / (|n| g). Always positive - the turn direction is carried
/// by angularRate, not by the sign of the radius.
/// @param[in] speed_mps Speed along the turn.
/// @param[in] load_factor Load factor of the turn; only its magnitude is used.
/// @returns The turn radius, always positive.
[[nodiscard]] double turnRadius(double speed_mps, double load_factor) noexcept;

/// @brief Turn rate for a coordinated turn at the given speed and load
/// factor: omega = n g / v.
/// @param[in] speed_mps Speed along the turn.
/// @param[in] load_factor Load factor of the turn; sign sets the turn direction.
/// @returns The signed turn rate.
[[nodiscard]] double angularRate(double speed_mps, double load_factor) noexcept;

}  // namespace detail
}  // namespace target
