#pragma once

#include <Eigen/Dense>

#include "guidance/controller_concept.hpp"
#include "guidance/pn_control_law.hpp"
#include "guidance/thrust_control_law.hpp"
#include "guidance/transverse_control_allocation.hpp"
#include "math/cartesian_state.hpp"

namespace guidance
{

struct PNControllerConfig
{
    double navigation_gain{3.0};
    ThrustControlConfig thrust_control_config{};
    TransverseControlAllocationConfig transverse_control_allocation_config{};
};

/// @brief Proportional-navigation controller: a PN acceleration command,
/// allocated to thrust, load factor and bank angle.
class PNController
{
public:
    /// @brief Construct a PN controller.
    /// @param[in] config The PN gain, thrust law and transverse allocation
    /// configuration.
    explicit PNController(PNControllerConfig const& config) noexcept;

    /// @brief Compute the commanded control for one guidance update.
    /// @details Below the thrust law's switch speed, PN and allocation are
    /// skipped and the vehicle is held wings-level (n=1, bank=0) through the
    /// boost phase.
    /// @param[in] target The target's Cartesian state.
    /// @param[in] interceptor The interceptor's Cartesian state.
    /// @param[in] dt The time since the previous update.
    /// @returns [thrust, load_factor, bank_angle_rad].
    [[nodiscard]] Eigen::Vector3d step(math::CartesianState const& target,
                                       math::CartesianState const& interceptor,
                                       double dt) const noexcept;

private:
    ProportionalNavigationControlLaw pn_control_law_;
    ThrustControlLaw thrust_control_law_;
    TransverseControlAllocation transverse_control_allocation_;
};

static_assert(GuidanceController<PNController>);

}  // namespace guidance
