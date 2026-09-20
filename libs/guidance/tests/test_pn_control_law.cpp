#include "guidance/pn_control_law.hpp"

#include "math/cartesian_state.hpp"

#include <gtest/gtest.h>

namespace guidance
{
namespace
{

constexpr double k_tol{1.0e-9};

/// @brief Cartesian state built from position and velocity.
[[nodiscard]] math::CartesianState make_cartesian(Eigen::Vector3d const& position_m,
                                                   Eigen::Vector3d const& velocity_mps)
{
    math::CartesianState state;
    state.position_m = position_m;
    state.velocity_mps = velocity_mps;
    return state;
}

class PNControlLawTest : public ::testing::Test
{
protected:
    ProportionalNavigationControlLaw law_{3.0};
};

TEST_F(PNControlLawTest, CollinearGeometryProducesZeroAcceleration)
{
    // Interceptor and target on the same line, closing directly: the LOS
    // rate is zero, so PN has no lateral error to correct.
    auto const interceptor =
        make_cartesian(Eigen::Vector3d{0.0, 0.0, 0.0}, Eigen::Vector3d{100.0, 0.0, 0.0});
    auto const target =
        make_cartesian(Eigen::Vector3d{1000.0, 0.0, 0.0}, Eigen::Vector3d{-50.0, 0.0, 0.0});

    auto const a_c = law_.step(target, interceptor);

    EXPECT_TRUE(a_c.isApprox(Eigen::Vector3d::Zero(), k_tol));
}

TEST_F(PNControlLawTest, AccelerationIsAlwaysPerpendicularToInterceptorVelocity)
{
    auto const interceptor =
        make_cartesian(Eigen::Vector3d{0.0, 0.0, 0.0}, Eigen::Vector3d{100.0, 0.0, 0.0});
    auto const target = make_cartesian(Eigen::Vector3d{1000.0, 100.0, 50.0},
                                       Eigen::Vector3d{-100.0, 10.0, -5.0});

    auto const a_c = law_.step(target, interceptor);

    EXPECT_NEAR(a_c.dot(interceptor.velocity_mps), 0.0, k_tol);
}

TEST_F(PNControlLawTest, AccelerationScalesLinearlyWithNavigationGain)
{
    auto const interceptor =
        make_cartesian(Eigen::Vector3d{0.0, 0.0, 0.0}, Eigen::Vector3d{100.0, 0.0, 0.0});
    auto const target =
        make_cartesian(Eigen::Vector3d{1000.0, 100.0, 0.0}, Eigen::Vector3d{-100.0, 0.0, 0.0});

    ProportionalNavigationControlLaw const double_gain{6.0};

    auto const a_c = law_.step(target, interceptor);
    auto const a_c_double = double_gain.step(target, interceptor);

    EXPECT_TRUE(a_c_double.isApprox(2.0 * a_c, k_tol));
}

TEST_F(PNControlLawTest, MatchesTheClosedFormTrueProportionalNavigationLaw)
{
    auto const interceptor =
        make_cartesian(Eigen::Vector3d{0.0, 0.0, 0.0}, Eigen::Vector3d{100.0, 0.0, 0.0});
    auto const target =
        make_cartesian(Eigen::Vector3d{1000.0, 100.0, 0.0}, Eigen::Vector3d{-100.0, 0.0, 0.0});

    Eigen::Vector3d const r = target.position_m - interceptor.position_m;
    Eigen::Vector3d const v_r = target.velocity_mps - interceptor.velocity_mps;
    double const range = r.norm();
    double const closing_speed = -r.dot(v_r) / range;
    Eigen::Vector3d const omega = r.cross(v_r) / (range * range);
    Eigen::Vector3d const v_hat = interceptor.velocity_mps.normalized();
    Eigen::Vector3d const expected = 3.0 * closing_speed * omega.cross(v_hat);

    auto const a_c = law_.step(target, interceptor);

    EXPECT_TRUE(a_c.isApprox(expected, k_tol));
}

TEST_F(PNControlLawTest, StaysFiniteWhenInterceptorVelocityIsZero)
{
    auto const interceptor =
        make_cartesian(Eigen::Vector3d{0.0, 0.0, 0.0}, Eigen::Vector3d::Zero());
    auto const target =
        make_cartesian(Eigen::Vector3d{1000.0, 100.0, 0.0}, Eigen::Vector3d{-100.0, 0.0, 0.0});

    EXPECT_TRUE(law_.step(target, interceptor).allFinite());
}

TEST_F(PNControlLawTest, StaysFiniteWhenRangeIsZero)
{
    auto const interceptor =
        make_cartesian(Eigen::Vector3d{500.0, 50.0, 0.0}, Eigen::Vector3d{100.0, 0.0, 0.0});
    auto const target =
        make_cartesian(Eigen::Vector3d{500.0, 50.0, 0.0}, Eigen::Vector3d{-100.0, 0.0, 0.0});

    EXPECT_TRUE(law_.step(target, interceptor).allFinite());
}

}  // namespace
}  // namespace guidance
