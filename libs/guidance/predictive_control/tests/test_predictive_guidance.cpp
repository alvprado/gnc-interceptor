#include "guidance/predictive_guidance_controller.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace guidance
{
namespace
{

TEST(PredictiveGuidanceTest, StartsWithShortHorizonAndRemainsFiniteWhenItGrows)
{
    PredictiveGuidanceControllerConfig config;
    config.boost_phase_switch_speed_mps = 33.33;
    PredictiveGuidanceController controller{config};
    math::CartesianState interceptor;
    interceptor.velocity_mps = {34.0, 0.0, 0.0};
    math::CartesianState target;
    target.position_m = {40.0, 20.0, 20.0};

    for (int i = 0; i < 6; ++i)
    {
        if (i == 3) target.position_m = {400.0, -200.0, 200.0};
        auto const control = controller.step(target, interceptor, 0.1);
        ASSERT_TRUE(control.allFinite());
        EXPECT_GE(control[1], config.min_load_factor);
        EXPECT_LE(control[1], config.max_load_factor);
        EXPECT_LE(std::abs(control[2]), config.max_bank_angle_rad);
        if (i == 2) EXPECT_GT(control[2], 0.0);
        if (i == 5) EXPECT_LT(control[2], 0.0);
    }
}

}  // namespace
}  // namespace guidance
