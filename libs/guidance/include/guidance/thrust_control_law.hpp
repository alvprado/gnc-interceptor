#pragma once

#include "guidance/model_parameters.hpp"
#include "math/cartesian_state.hpp"

namespace guidance
{

/// @brief Configuration for ThrustControlLaw.
struct ThrustControlConfig
{
    double switch_speed_mps{90.0};   ///< Speed for switching from boost to trim.
    double max_thrust_n{150.0};      ///< Thrust commanded during boost, in N.
    ModelParameters vehicle{};       ///< Vehicle model used by the trim law.
};

/// @brief Boost-then-trim thrust law: max thrust below switch_speed_mps,
/// then feedforward thrust to hold speed (cancelling drag and the gravity
/// component along the flight path).
class ThrustControlLaw
{
public:
    /// @brief Construct a thrust control law.
    /// @param[in] config The switch speed, boost thrust and vehicle estimate.
    explicit ThrustControlLaw(ThrustControlConfig const& config) noexcept;

    /// @brief Compute the commanded thrust for one guidance update.
    /// @param[in] interceptor The interceptor's Cartesian state.
    /// @returns The commanded thrust, in N.
    [[nodiscard]] double step(math::CartesianState const& interceptor) const noexcept;

    /// @brief The speed at which this law switches from boost to trim.
    /// @returns The switch speed, in m/s.
    [[nodiscard]] double switchSpeedMps() const noexcept;

private:
    ThrustControlConfig config_;
};

}  // namespace guidance
