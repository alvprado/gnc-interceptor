#include "target/geometry.hpp"

#include "math/constants.hpp"

#include <cmath>

namespace target
{
namespace detail
{

PlaneBasis planeBasis(Eigen::Vector3d const& normal, Eigen::Vector3d const& reference)
{
    Eigen::Vector3d const n_hat = normal.normalized();
    Eigen::Vector3d const u = (reference - reference.dot(n_hat) * n_hat).normalized();
    return PlaneBasis{u, n_hat.cross(u), n_hat};
}

double turnRadius(double speed_mps, double load_factor) noexcept
{
    return speed_mps * speed_mps / (std::abs(load_factor) * math::k_gravity_mps2);
}

double angularRate(double speed_mps, double load_factor) noexcept
{
    return load_factor * math::k_gravity_mps2 / speed_mps;
}

}  // namespace detail
}  // namespace target
