#include "math/angles.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

namespace math
{
namespace
{

constexpr double k_tol{1.0e-12};

/// @brief Unit velocity direction for a heading/flight-path angle pair, per
/// the convention shared with UAV3DofModel::toVehicleState.
[[nodiscard]] Eigen::Vector3d velocityDirection(double heading_rad, double flight_path_angle_rad)
{
    double const cos_fpa = std::cos(flight_path_angle_rad);
    return Eigen::Vector3d{cos_fpa * std::cos(heading_rad), cos_fpa * std::sin(heading_rad),
                           std::sin(flight_path_angle_rad)};
}

TEST(OrientationTest, ZeroAnglesGiveIdentity)
{
    auto const q = attitudeFromHeadingPitchBank(0.0, 0.0, 0.0);
    EXPECT_TRUE(q.isApprox(Eigen::Quaterniond::Identity()));
}

TEST(OrientationTest, ResultIsAlwaysUnitNorm)
{
    for (double heading : {0.0, 0.7, -1.3, std::numbers::pi})
    {
        for (double fpa : {0.0, 0.4, -0.9})
        {
            for (double bank : {0.0, 0.5, -2.0, std::numbers::pi})
            {
                auto const q = attitudeFromHeadingPitchBank(heading, fpa, bank);
                EXPECT_NEAR(q.norm(), 1.0, k_tol);
            }
        }
    }
}

TEST(OrientationTest, ForwardAxisMatchesVelocityDirectionAtZeroBank)
{
    for (double heading : {0.0, 0.7, -1.3, std::numbers::pi / 2.0})
    {
        for (double fpa : {0.0, 0.4, -0.9, std::numbers::pi / 2.0 - 0.01})
        {
            auto const q = attitudeFromHeadingPitchBank(heading, fpa, 0.0);
            Eigen::Vector3d const forward = q * Eigen::Vector3d::UnitX();
            EXPECT_TRUE(forward.isApprox(velocityDirection(heading, fpa), k_tol));
        }
    }
}

TEST(OrientationTest, BankDoesNotChangeTheForwardAxis)
{
    constexpr double heading{0.6};
    constexpr double fpa{-0.3};
    Eigen::Vector3d const expected_forward = velocityDirection(heading, fpa);

    for (double bank : {0.0, 0.5, 1.7, -2.5, std::numbers::pi})
    {
        auto const q = attitudeFromHeadingPitchBank(heading, fpa, bank);
        EXPECT_TRUE((q * Eigen::Vector3d::UnitX()).isApprox(expected_forward, k_tol));
    }
}

TEST(OrientationTest, BankRotatesTheUpAxisAboutForward)
{
    // heading = fpa = 0: forward is +x, up starts as +z. A +90deg roll about
    // +x rotates +z toward -y (right-handed rotation), pinning the sign
    // convention for bank.
    auto const q = attitudeFromHeadingPitchBank(0.0, 0.0, std::numbers::pi / 2.0);
    Eigen::Vector3d const up = q * Eigen::Vector3d::UnitZ();
    EXPECT_TRUE(up.isApprox(Eigen::Vector3d{0.0, -1.0, 0.0}, k_tol));
}

TEST(OrientationTest, EulerAnglesFromAttitudeInvertsAttitudeFromHeadingPitchBank)
{
    // Avoids exactly +-pi, where atan2's (-pi, pi] wraparound makes exact
    // equality brittle without being a meaningfully different case.
    for (double heading : {0.0, 0.7, -1.3, std::numbers::pi / 2.0, 3.0})
    {
        for (double fpa : {0.0, 0.4, -0.9, std::numbers::pi / 2.0 - 0.01})
        {
            for (double bank : {0.0, 0.5, -2.0, 1.7, 2.9})
            {
                auto const q = attitudeFromHeadingPitchBank(heading, fpa, bank);
                auto const recovered = eulerAnglesFromAttitude(q);

                EXPECT_NEAR(recovered.heading_rad, heading, k_tol);
                EXPECT_NEAR(recovered.flight_path_angle_rad, fpa, k_tol);
                EXPECT_NEAR(recovered.bank_rad, bank, k_tol);
            }
        }
    }
}

}  // namespace
}  // namespace math
