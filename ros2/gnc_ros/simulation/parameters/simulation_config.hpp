#pragma once

#include <functional>

#include "math/integrators.hpp"
#include "simulation/uav_3dof_model.hpp"

namespace rclcpp
{
class Node;
}

namespace gnc_ros
{

/// @brief Integration scheme for simulation
enum class IntegrationMethod
{
    RK4,
    Heun,
    Euler,
};

/// @brief Simulation timestep and integration policy.
struct SimConfig
{
    using Model = simulation::UAV3DofModel;
    using Integrator = std::function<Model::StateVec(
        Model const&, Model::StateVec const&, Model::ControlVec const&, double)>;

    double dt_s{0.01};
    Integrator integrator{math::RK4Step{}};
};

/// @brief Declare and read the simulation's startup-only ROS parameters.
/// @details Invalid values fall back to the defaults with a warning.
[[nodiscard]] SimConfig readSimulationConfig(rclcpp::Node& node);

}  // namespace gnc_ros
