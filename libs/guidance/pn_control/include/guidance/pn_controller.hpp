#pragma once

#include <Eigen/Dense>
#include <numbers>

#include "guidance/controller_concept.hpp"
#include "guidance/model_parameters.hpp"
#include "guidance/pn_control_law.hpp"
#include "guidance/thrust_control_law.hpp"
#include "guidance/transverse_control_allocation.hpp"
#include "math/cartesian_state.hpp"

namespace guidance
{

/// @brief Configuration for PNController.
struct PNControllerConfig
{
    double navigation_gain{3.0};                 ///< PN navigation gain.
    double boost_phase_switch_speed_mps{90.0};   ///< Speed below which boost thrust is commanded.
    double boost_phase_thrust_n{150.0};          ///< Thrust commanded during the boost phase, in N.
    double min_load_factor{-3.0};                ///< Minimum load factor.
    double max_load_factor{9.0};                 ///< Maximum load factor.
    double max_bank_angle_rad{std::numbers::pi};  ///< Maximum |bank angle| in rad.
    ModelParameters vehicle{};                   ///< Vehicle model used by the trim thrust law.
};

/// @brief Proportional-navigation controller: a PN acceleration command,
/// allocated to thrust, load factor and bank angle.
class PNController
{
public:
    /// @brief Construct a PN controller.
    /// @param[in] config The PN gain, boost phase, vehicle and transverse
    /// allocation configuration; the individual configs for the PN control
    /// law, thrust control law and transverse allocation are built from it.
    explicit PNController(PNControllerConfig const& config) noexcept;

    /// @brief Compute the commanded control for one guidance update.
    /// @details Below boost_phase_switch_speed_mps, PN and allocation are
    /// skipped, boost_phase_thrust_n is commanded, and the vehicle is held
    /// wings-level (n=1, bank=0) through the boost phase.
    /// @param[in] target The target's Cartesian state.
    /// @param[in] interceptor The interceptor's Cartesian state.
    /// @param[in] dt The time since the previous update.
    /// @returns [thrust, load_factor, bank_angle_rad].
    [[nodiscard]] Eigen::Vector3d step(math::CartesianState const& target,
                                       math::CartesianState const& interceptor,
                                       double dt) const noexcept;

private:
    PNControllerConfig config_;
    ProportionalNavigationControlLaw pn_control_law_;
    ThrustControlLaw thrust_control_law_;
    TransverseControlAllocation transverse_control_allocation_;
};

static_assert(GuidanceController<PNController>);

}  // namespace guidance
