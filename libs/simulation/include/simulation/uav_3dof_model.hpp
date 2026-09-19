#pragma once

#include "math/constants.hpp"

#include <Eigen/Dense>

#include <numbers>

namespace simulation
{

/// @brief Physical parameters of the 3-DOF point-mass UAV model.
struct UAV3DofModelParams
{
    double mass_kg{5.0};                          ///< Vehicle mass in kg.
    double rho_kgpm3{math::air_density_kgpm3};  ///< Air density in kg/m^3.
    double frontal_area_m2{0.1};                  ///< Reference frontal area in m^2.
    double drag_coeff{0.08};                      ///< Dimensionless drag coefficient.
};

/// @brief Operational envelope of the 3-DOF point-mass UAV model.
struct UAV3DofModelLimits
{
    double min_thrust_n{0.0};                     ///< Minimum thrust in N.
    double max_thrust_n{150.0};                   ///< Maximum thrust in N.
    double min_load_factor{-3.0};                 ///< Minimum load factor.
    double max_load_factor{9.0};                  ///< Maximum load factor.
    double max_bank_angle_rad{std::numbers::pi};  ///< Maximum |bank angle| in rad.
    double min_speed_mps{1.0};                    ///< Minimum speed in m/s.
    double max_speed_mps{300.0};                  ///< Maximum speed in m/s.
};

/// @brief 3-DOF point-mass UAV model supplying the continuous dynamics
/// ẋ = f(x, u) and the limits of its own state and control.
/// @details The state is x = [x, y, z, v, psi, gamma]^T, where x, y, z is the
/// position, v the speed, psi the heading angle and gamma the flight-path angle
/// of the velocity vector. The control is u = [T, n, alpha]^T, where T is the
/// thrust, n the load factor (lift / mg) and alpha the bank angle.
class UAV3DofModel
{
public:
    using StateVec = Eigen::Vector<double, 6>;    ///< [x, y, z, v, psi, gamma]^T.
    using ControlVec = Eigen::Vector<double, 3>;  ///< [T, n, alpha]^T.

    /// @brief Construct the model from its physical parameters and limits.
    /// @param[in] params The parameters the dynamics are evaluated with.
    /// @param[in] limits The envelope the state and control are clamped to.
    explicit UAV3DofModel(UAV3DofModelParams const& params,
                          UAV3DofModelLimits const& limits = {});

    /// @brief Evaluate the continuous dynamics ẋ = f(x, u).
    /// @param[in] state The state to evaluate the dynamics at.
    /// @param[in] control The control to evaluate the dynamics at.
    /// @returns The state derivative ẋ.
    [[nodiscard]] StateVec operator()(StateVec const& state,
                                      ControlVec const& control) const noexcept;

    /// @brief Clamp a control to the actuator envelope.
    /// @param[in] control The commanded control.
    /// @returns The control limited to the envelope.
    [[nodiscard]] ControlVec clampControl(ControlVec const& control) const noexcept;

    /// @brief Clamp a state to the flight envelope, wrapping heading and
    /// flight-path angle into (-pi, pi].
    /// @param[in] state The state to limit.
    /// @returns The state limited to the envelope.
    [[nodiscard]] StateVec clampState(StateVec const& state) const noexcept;

private:
    UAV3DofModelParams params_;
    UAV3DofModelLimits limits_;
};

}  // namespace simulation
