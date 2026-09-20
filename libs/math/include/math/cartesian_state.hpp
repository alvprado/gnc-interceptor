#pragma once

#include <Eigen/Dense>

namespace math
{

/// @brief Cartesian state of a point at an instant: position, velocity and
/// acceleration in the inertial frame.
struct CartesianState
{
    Eigen::Vector3d position_m{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity_mps{Eigen::Vector3d::Zero()};
    Eigen::Vector3d acceleration_mps2{Eigen::Vector3d::Zero()};
};

}  // namespace math
