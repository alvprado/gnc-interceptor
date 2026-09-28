#include "guidance/thrust_control_law.hpp"

#include "math/constants.hpp"
#include "math/state_types.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace guidance
{
namespace
{

constexpr double k_tol{1.0e-9};
constexpr double k_test_speed_mps{100.0};

/// @brief Cartesian state built from a velocity, at the origin.
[[nodiscard]] math::CartesianState make_cartesian(Eigen::Vector3d const& velocity_mps)
{
    math::CartesianState state;
    state.velocity_mps = velocity_mps;
    return state;
}

class ThrustControlLawTest : public ::testing::Test
{
protected:
    ThrustControlConfig config_{};
    ThrustControlLaw law_{config_};
};

TEST_F(ThrustControlLawTest, TrimThrustExactlyCancelsDragInLevelFlight)
{
    double const speed = k_test_speed_mps;
    auto const interceptor = make_cartesian(Eigen::Vector3d{speed, 0.0, 0.0});

    auto const& vehicle = config_.vehicle;
    double const expected_drag =
        0.5 * vehicle.rho_kgpm3 * vehicle.frontal_area_m2 * vehicle.drag_coeff * speed * speed;

    EXPECT_NEAR(law_.step(interceptor), expected_drag, k_tol);
}

TEST_F(ThrustControlLawTest, TrimThrustAddsWeightComponentWhenClimbing)
{
    double const speed = k_test_speed_mps;
    double const climb_rate = 0.3 * speed;
    double const forward_rate = std::sqrt(speed * speed - climb_rate * climb_rate);
    auto const interceptor = make_cartesian(Eigen::Vector3d{forward_rate, 0.0, climb_rate});

    auto const& vehicle = config_.vehicle;
    double const drag =
        0.5 * vehicle.rho_kgpm3 * vehicle.frontal_area_m2 * vehicle.drag_coeff * speed * speed;
    double const expected = drag + math::gravity_mps2 * (climb_rate / speed) * vehicle.mass_kg;

    EXPECT_NEAR(law_.step(interceptor), expected, k_tol);
    EXPECT_GT(law_.step(interceptor), drag) << "climbing needs more than just drag cancellation";
}

TEST_F(ThrustControlLawTest, TrimThrustSubtractsWeightComponentWhenDiving)
{
    double const speed = k_test_speed_mps;
    double const dive_rate = 0.3 * speed;
    double const forward_rate = std::sqrt(speed * speed - dive_rate * dive_rate);
    auto const interceptor = make_cartesian(Eigen::Vector3d{forward_rate, 0.0, -dive_rate});

    auto const& vehicle = config_.vehicle;
    double const drag =
        0.5 * vehicle.rho_kgpm3 * vehicle.frontal_area_m2 * vehicle.drag_coeff * speed * speed;

    EXPECT_LT(law_.step(interceptor), drag) << "diving needs less than just drag cancellation";
}

}  // namespace
}  // namespace guidance
