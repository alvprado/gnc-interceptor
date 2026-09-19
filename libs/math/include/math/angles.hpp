#pragma once

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

}  // namespace math
