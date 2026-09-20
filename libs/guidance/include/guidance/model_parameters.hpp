#pragma once

#include "math/constants.hpp"

namespace guidance
{

/// @brief The interceptor's mass and aerodynamic properties, as known to
/// the guidance stack.
struct ModelParameters
{
    double mass_kg{2.5};                        ///< Vehicle mass in kg.
    double rho_kgpm3{math::air_density_kgpm3};  ///< Air density in kg/m^3.
    double frontal_area_m2{0.1};                ///< Reference frontal area in m^2.
    double drag_coeff{0.08};                    ///< Dimensionless drag coefficient.
};

}  // namespace guidance
