#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <chrono>

namespace math
{

/// @brief Simulation time, in seconds since the run started.
using Timestamp = std::chrono::duration<double>;

/// @brief Cartesian state of a point at an instant: position, velocity and
/// acceleration in the inertial frame.
struct CartesianState
{
    Eigen::Vector3d position_m{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity_mps{Eigen::Vector3d::Zero()};
    Eigen::Vector3d acceleration_mps2{Eigen::Vector3d::Zero()};
};

/// @brief Cartesian state of a vehicle plus its body attitude.
struct VehicleState
{
    CartesianState cartesian{};
    Eigen::Quaterniond attitude{Eigen::Quaterniond::Identity()};
};
}  // namespace math
