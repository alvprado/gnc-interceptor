#include "target/maneuvers.hpp"

#include "math/constants.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

namespace target
{
namespace
{

constexpr double k_tight_tol{1.0e-9};

/// @brief Central-difference velocity and acceleration, independent of the
/// closed-form derivatives under test.
/// @tparam Trajectory_T A type satisfying TargetTrajectory.
template <TargetTrajectory Trajectory_T>
struct NumericalDerivative
{
    Eigen::Vector3d velocity_mps;
    Eigen::Vector3d acceleration_mps2;
};

template <TargetTrajectory Trajectory_T>
[[nodiscard]] NumericalDerivative<Trajectory_T> numericalDerivativeAt(Trajectory_T const& traj,
                                                                      double time_s)
{
    constexpr double h{1.0e-6};
    auto const plus = traj.evaluateTargetStateAt(time_s + h);
    auto const minus = traj.evaluateTargetStateAt(time_s - h);
    return {(plus.position_m - minus.position_m) / (2.0 * h),
           (plus.velocity_mps - minus.velocity_mps) / (2.0 * h)};
}

/// @brief Assert the closed-form derivatives agree with a numerical estimate.
/// @tparam Trajectory_T A type satisfying TargetTrajectory.
template <TargetTrajectory Trajectory_T>
void expectDerivativesAreConsistent(Trajectory_T const& traj, double time_s,
                                    double tol = 1.0e-4)
{
    auto const analytic = traj.evaluateTargetStateAt(time_s);
    auto const numeric = numericalDerivativeAt(traj, time_s);

    EXPECT_LT((analytic.velocity_mps - numeric.velocity_mps).norm(), tol)
        << "velocity disagrees with the numerical derivative at t = " << time_s;
    EXPECT_LT((analytic.acceleration_mps2 - numeric.acceleration_mps2).norm(), tol)
        << "acceleration disagrees with the numerical derivative at t = " << time_s;
}


TEST(ConstantVelocityTest, PositionIsLinearInTime)
{
    ConstantVelocity const traj{{10.0, -5.0, 100.0}, {50.0, 20.0, -3.0}};

    auto const s0 = traj.evaluateTargetStateAt(0.0);
    auto const s5 = traj.evaluateTargetStateAt(5.0);

    EXPECT_TRUE(s0.position_m.isApprox(Eigen::Vector3d(10.0, -5.0, 100.0)));
    EXPECT_TRUE(s5.position_m.isApprox(Eigen::Vector3d(260.0, 95.0, 85.0)));
}

TEST(ConstantVelocityTest, VelocityAndAccelerationAreConstant)
{
    ConstantVelocity const traj{{0.0, 0.0, 0.0}, {50.0, 20.0, -3.0}};

    for (double const t : {0.0, 1.0, 100.0})
    {
        auto const s = traj.evaluateTargetStateAt(t);
        EXPECT_TRUE(s.velocity_mps.isApprox(Eigen::Vector3d(50.0, 20.0, -3.0)));
        EXPECT_TRUE(s.acceleration_mps2.isApprox(Eigen::Vector3d::Zero()));
    }
}

TEST(ConstantVelocityTest, DerivativesMatchNumericalEstimate)
{
    ConstantVelocity const traj{{1.0, 2.0, 3.0}, {12.0, -7.0, 4.0}};
    for (double const t : {0.0, 2.5, 30.0})
    {
        expectDerivativesAreConsistent(traj, t);
    }
}


class CircleTest : public ::testing::Test
{
protected:
    // Radius and rate are the documented R = v^2 / (n g), omega = n g / v,
    // recomputed independently of the implementation to check it.
    Eigen::Vector3d const center_{100.0, 200.0, 0.0};
    double const speed_mps_{150.0};
    double const load_factor_{3.0};
    double const radius_m_{speed_mps_ * speed_mps_ / (load_factor_ * math::k_gravity_mps2)};
    double const rate_rps_{load_factor_ * math::k_gravity_mps2 / speed_mps_};
    Circle const traj_{center_, speed_mps_, load_factor_, Eigen::Vector3d(0, 0, 1),
                       Eigen::Vector3d(1, 0, 0)};
};

TEST_F(CircleTest, StartsAtTheReferenceDirection)
{
    auto const s0 = traj_.evaluateTargetStateAt(0.0);
    EXPECT_TRUE(s0.position_m.isApprox(center_ + Eigen::Vector3d(radius_m_, 0, 0)));
}

TEST_F(CircleTest, RadiusIsExactAtEveryInstant)
{
    for (double t = 0.0; t <= 100.0; t += 0.37)
    {
        auto const s = traj_.evaluateTargetStateAt(t);
        EXPECT_NEAR((s.position_m - center_).norm(), radius_m_, k_tight_tol)
            << "radius drifted at t = " << t;
    }
}

TEST_F(CircleTest, SpeedIsConstantAndEqualsRadiusTimesRate)
{
    for (double const t : {0.0, 1.3, 7.0, 20.0})
    {
        auto const s = traj_.evaluateTargetStateAt(t);
        EXPECT_NEAR(s.velocity_mps.norm(), radius_m_ * rate_rps_, k_tight_tol);
    }
}

TEST_F(CircleTest, AccelerationIsCentripetal)
{
    auto const s = traj_.evaluateTargetStateAt(2.5);
    Eigen::Vector3d const expected = -rate_rps_ * rate_rps_ * (s.position_m - center_);
    EXPECT_TRUE(s.acceleration_mps2.isApprox(expected, k_tight_tol));
}

TEST_F(CircleTest, PositionAndVelocityAreOrthogonal)
{
    // The tangent to a circle is always normal to the radius.
    for (double const t : {0.0, 4.0, 9.5})
    {
        auto const s = traj_.evaluateTargetStateAt(t);
        EXPECT_NEAR((s.position_m - center_).dot(s.velocity_mps), 0.0, 1.0e-6);
    }
}

TEST_F(CircleTest, DerivativesMatchNumericalEstimate)
{
    for (double const t : {0.0, 1.0, 5.0, 15.0})
    {
        expectDerivativesAreConsistent(traj_, t);
    }
}

TEST(CircleGeneralPlaneTest, HoldsExactRadiusForAnArbitraryNonAxisAlignedPlane)
{
    // A circle whose plane is neither horizontal nor vertical exercises the
    // general Gram-Schmidt path in planeBasis rather than an axis-aligned one.
    double const speed_mps = 175.0, load_factor = 2.0;
    double const radius_m =
        speed_mps * speed_mps / (load_factor * math::k_gravity_mps2);
    Circle const traj{{0, 0, 0}, speed_mps, load_factor, Eigen::Vector3d(0.3, 0.5, 0.8),
                      Eigen::Vector3d(1, 0, 0)};

    for (double t = 0.0; t <= 40.0; t += 1.1)
    {
        auto const s = traj.evaluateTargetStateAt(t);
        EXPECT_NEAR(s.position_m.norm(), radius_m, 1.0e-8);
        expectDerivativesAreConsistent(traj, t);
    }
}

TEST(CircleGeneralPlaneTest, SignOfLoadFactorSetsTheTurnDirectionFromTheSameStart)
{
    // R must stay positive under a sign flip: only the turn direction (the
    // sign of the angular rate) should follow the sign of n.
    Circle const positive{
        {0, 0, 0}, 150.0, 3.0, Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(1, 0, 0)};
    Circle const negative{
        {0, 0, 0}, 150.0, -3.0, Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(1, 0, 0)};

    EXPECT_TRUE(positive.evaluateTargetStateAt(0.0).position_m.isApprox(
        negative.evaluateTargetStateAt(0.0).position_m))
        << "both must start at the same point regardless of turn direction";

    auto const p = positive.evaluateTargetStateAt(0.05);
    auto const n = negative.evaluateTargetStateAt(0.05);
    EXPECT_LT(p.position_m.y() * n.position_m.y(), 0.0)
        << "the two turns must diverge to opposite sides of the reference direction";
}


class FigureEightTest : public ::testing::Test
{
protected:
    Eigen::Vector3d const center_{0.0, 0.0, 500.0};
    double const length_m_{800.0};
    double const width_m_{400.0};
    double const rate_rps_{0.1};
    FigureEight const traj_{center_, length_m_, width_m_, rate_rps_, Eigen::Vector3d(0, 0, 1),
                            Eigen::Vector3d(1, 0, 0)};
};

TEST_F(FigureEightTest, PassesThroughTheCenterFourTimesPerPeriod)
{
    // sin(theta) and sin(2 theta) are both zero at theta = 0, pi, 2pi, ... -
    // the crossing point at the middle of the eight.
    double const period = 2.0 * std::numbers::pi / rate_rps_;
    for (double const theta : {0.0, std::numbers::pi, 2.0 * std::numbers::pi})
    {
        auto const s = traj_.evaluateTargetStateAt(theta / rate_rps_);
        EXPECT_NEAR((s.position_m - center_).norm(), 0.0, 1.0e-6) << "theta = " << theta;
    }
    static_cast<void>(period);
}

TEST_F(FigureEightTest, StaysWithinItsBoundingExtent)
{
    for (double t = 0.0; t <= 2.0 * std::numbers::pi / rate_rps_; t += 0.2)
    {
        auto const s = traj_.evaluateTargetStateAt(t);
        Eigen::Vector3d const offset = s.position_m - center_;
        EXPECT_LE(std::abs(offset.x()), 0.5 * length_m_ + 1.0e-9);
        EXPECT_LE(std::abs(offset.y()), 0.5 * width_m_ + 1.0e-9);
    }
}

TEST_F(FigureEightTest, DerivativesMatchNumericalEstimate)
{
    for (double const t : {0.0, 3.0, 12.0, 25.0})
    {
        expectDerivativesAreConsistent(traj_, t);
    }
}


class HelixTest : public ::testing::Test
{
protected:
    // Radius, rate and climb rate follow from (speed, load factor, climb
    // angle) via the documented relations, recomputed independently here.
    Eigen::Vector3d const center_{0.0, 0.0, 0.0};
    double const speed_mps_{200.0};
    double const load_factor_{2.0};
    double const climb_angle_rad_{15.0 * std::numbers::pi / 180.0};
    double const horizontal_speed_mps_{speed_mps_ * std::cos(climb_angle_rad_)};
    double const radius_m_{horizontal_speed_mps_ * horizontal_speed_mps_ /
                           (load_factor_ * math::k_gravity_mps2)};
    double const rate_rps_{load_factor_ * math::k_gravity_mps2 / horizontal_speed_mps_};
    double const climb_rate_mps_{speed_mps_ * std::sin(climb_angle_rad_)};
    Helix const traj_{center_,        speed_mps_,       load_factor_, climb_angle_rad_,
                      Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(1, 0, 0)};
};

TEST_F(HelixTest, AltitudeIsLinearInTime)
{
    for (double const t : {0.0, 2.0, 5.0, 30.0})
    {
        auto const s = traj_.evaluateTargetStateAt(t);
        EXPECT_NEAR(s.position_m.z(), climb_rate_mps_ * t, k_tight_tol);
    }
}

TEST_F(HelixTest, PlanarRadiusIsExactAtEveryInstant)
{
    for (double t = 0.0; t <= 50.0; t += 0.9)
    {
        auto const s = traj_.evaluateTargetStateAt(t);
        double const planar_radius =
            std::sqrt(s.position_m.x() * s.position_m.x() + s.position_m.y() * s.position_m.y());
        EXPECT_NEAR(planar_radius, radius_m_, k_tight_tol) << "drifted at t = " << t;
    }
}

TEST_F(HelixTest, VerticalSpeedIsConstantAndEqualsTheClimbRate)
{
    for (double const t : {0.0, 4.0, 11.0})
    {
        auto const s = traj_.evaluateTargetStateAt(t);
        EXPECT_NEAR(s.velocity_mps.z(), climb_rate_mps_, k_tight_tol);
    }
}

TEST_F(HelixTest, TotalSpeedEqualsTheCommandedSpeed)
{
    for (double const t : {0.0, 4.0, 11.0})
    {
        auto const s = traj_.evaluateTargetStateAt(t);
        EXPECT_NEAR(s.velocity_mps.norm(), speed_mps_, k_tight_tol);
    }
}

TEST_F(HelixTest, AccelerationHasNoVerticalComponent)
{
    // Constant climb rate: the axial acceleration is exactly zero, only the
    // in-plane centripetal term is non-zero.
    auto const s = traj_.evaluateTargetStateAt(6.0);
    EXPECT_NEAR(s.acceleration_mps2.z(), 0.0, k_tight_tol);
    EXPECT_GT(s.acceleration_mps2.head<2>().norm(), 0.0);
}

TEST_F(HelixTest, DerivativesMatchNumericalEstimate)
{
    for (double const t : {0.0, 3.0, 9.0, 22.0})
    {
        expectDerivativesAreConsistent(traj_, t);
    }
}

TEST(HelixDegenerateTest, ZeroClimbAngleReducesToACircle)
{
    Helix const helix{
        {0, 0, 0}, 220.0, 2.5, 0.0, Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(1, 0, 0)};
    Circle const circle{
        {0, 0, 0}, 220.0, 2.5, Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(1, 0, 0)};

    for (double const t : {0.0, 1.0, 5.0, 10.0})
    {
        auto const h = helix.evaluateTargetStateAt(t);
        auto const c = circle.evaluateTargetStateAt(t);
        EXPECT_TRUE(h.position_m.isApprox(c.position_m, k_tight_tol));
        EXPECT_TRUE(h.velocity_mps.isApprox(c.velocity_mps, k_tight_tol));
    }
}

}  // namespace
}  // namespace target
