#pragma once

#include "guidance/model_parameters.hpp"
#include "math/state_types.hpp"

namespace guidance
{

/// @brief Configuration for ThrustControlLaw.
struct ThrustControlConfig
{
    ModelParameters vehicle{};  ///< Vehicle model used by the trim law.
};

/// @brief Trim thrust law: feedforward thrust to hold speed (cancelling drag
/// and the gravity component along the flight path).
class ThrustControlLaw
{
public:
    /// @brief Construct a thrust control law.
    /// @param[in] config The vehicle estimate used by the trim law.
    explicit ThrustControlLaw(ThrustControlConfig const& config) noexcept;

    /// @brief Feedforward thrust that exactly cancels drag and the gravity component along the
    /// flight path (v_dot = 0).
    /// @param[in] interceptor The interceptor's Cartesian state.
    /// @returns The trim thrust, in N.
    [[nodiscard]] double step(math::CartesianState const& interceptor) const noexcept;

private:
    ThrustControlConfig config_;
};

}  // namespace guidance
