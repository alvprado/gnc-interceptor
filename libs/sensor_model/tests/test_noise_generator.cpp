#include "sensor_model/noise_generator.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <type_traits>

namespace sensor_model
{
namespace
{

constexpr int k_num_samples{200000};

TEST(GaussianNoiseGeneratorTest, IsMoveOnly)
{
    static_assert(!std::is_copy_constructible_v<GaussianNoiseGenerator>);
    static_assert(!std::is_copy_assignable_v<GaussianNoiseGenerator>);
    static_assert(std::is_move_constructible_v<GaussianNoiseGenerator>);
    static_assert(std::is_move_assignable_v<GaussianNoiseGenerator>);
}

TEST(GaussianNoiseGeneratorTest, NonPositiveStddevIsClampedInsteadOfThrowing)
{
    GaussianNoiseGenerator zero_stddev{0.0, 0.0, 1};
    GaussianNoiseGenerator negative_stddev{0.0, -5.0, 2};

    EXPECT_TRUE(std::isfinite(zero_stddev.sample()));
    EXPECT_TRUE(std::isfinite(negative_stddev.sample()));
}

TEST(GaussianNoiseGeneratorTest, SameSeedReproducesTheSameSequence)
{
    GaussianNoiseGenerator a{1.0, 2.0, 42};
    GaussianNoiseGenerator b{1.0, 2.0, 42};

    for (int i = 0; i < 10; ++i)
    {
        EXPECT_DOUBLE_EQ(a.sample(), b.sample());
    }
}

TEST(GaussianNoiseGeneratorTest, DifferentSeedsDiverge)
{
    GaussianNoiseGenerator a{0.0, 1.0, 1};
    GaussianNoiseGenerator b{0.0, 1.0, 2};

    EXPECT_NE(a.sample(), b.sample());
}

TEST(GaussianNoiseGeneratorTest, SampleStatisticsMatchConfiguredMeanAndStddev)
{
    constexpr double mean{5.0};
    constexpr double stddev{2.0};
    GaussianNoiseGenerator gen{mean, stddev, 7};

    double sum{0.0};
    double sum_sq{0.0};
    for (int i = 0; i < k_num_samples; ++i)
    {
        double const s = gen.sample();
        sum += s;
        sum_sq += s * s;
    }

    double const sample_mean = sum / k_num_samples;
    double const sample_var = sum_sq / k_num_samples - sample_mean * sample_mean;

    // Loose bounds: this is a statistical check, not an exact one.
    EXPECT_NEAR(sample_mean, mean, 0.05);
    EXPECT_NEAR(std::sqrt(sample_var), stddev, 0.05);
}

}  // namespace
}  // namespace sensor_model
