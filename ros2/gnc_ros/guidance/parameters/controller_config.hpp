#pragma once

#include <numbers>
#include <variant>

#include "guidance/model_parameters.hpp"
#include "guidance/pn_controller.hpp"
#include "guidance/predictive_guidance_controller.hpp"

namespace rclcpp
{
class Node;
}

namespace gnc_ros
{

enum class ControllerType
{
    ProportionalNavigation,
    Predictive,
};

/// @brief Guidance controller construction parameters; only fields used by the selected
/// controller type are read.
struct ControllerConfig
{
    ControllerType type{ControllerType::ProportionalNavigation};

    // Shared by both controllers.
    guidance::ModelParameters vehicle{};
    double min_load_factor{-3.0};
    double max_load_factor{9.0};
    double max_bank_angle_rad{std::numbers::pi};
    double boost_phase_switch_speed_mps{90.0};
    double boost_phase_thrust_n{150.0};

    // Proportional navigation.
    double navigation_gain{3.0};

    // Predictive (iLQR).
    int horizon{50};
    int min_horizon{5};
    double dt{0.1};
    double load_factor_rate_weight{2.0 / (9.0 * 9.0)};
    double bank_rate_weight{2.0 / (std::numbers::pi * std::numbers::pi)};
    double final_interception_weight{10.0};
    double running_interception_weight{10.0};
    double d_scale_time_constant_s{1.0};
};

/// @brief Type-erasing wrapper holding whichever guidance::GuidanceController is selected at
/// runtime
class GuidanceController
{
public:
    using Controller = std::variant<guidance::PNController, guidance::PredictiveGuidanceController>;

    explicit GuidanceController(Controller controller);

    /// @returns [thrust, load_factor, bank_angle_rad].
    [[nodiscard]] Eigen::Vector3d step(math::CartesianState const& target,
                                       math::CartesianState const& interceptor, double dt);

private:
    Controller controller_;
};

static_assert(guidance::GuidanceController<GuidanceController>);

/// @brief Declare and read the selected controller's startup-only ROS parameters.
[[nodiscard]] ControllerConfig readControllerConfig(rclcpp::Node& node);

/// @brief Construct the selected core controller from its configuration.
[[nodiscard]] GuidanceController makeGuidanceController(ControllerConfig const& config);

}  // namespace gnc_ros
