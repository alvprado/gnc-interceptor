#pragma once

#include <algorithm>
#include <random>

namespace sensor_model
{

/// @brief Floor a non-positive stddev is clamped to, so
/// std::normal_distribution's precondition (0 < stddev) always holds.
inline constexpr double fallback_min_stddev{1.0e-9};

/// @brief Independent Gaussian noise source: samples are i.i.d. N(mean, stddev^2).
/// @details Not copyable: copying would duplicate the engine and distribution
/// state, so two copies would emit the identical sequence from that point on.
class GaussianNoiseGenerator
{
public:
    /// @brief Construct a noise source.
    /// @param[in] mean The distribution mean.
    /// @param[in] stddev The distribution standard deviation; clamped up to
    /// fallback_min_stddev if not positive.
    /// @param[in] seed The engine seed; defaults to a nondeterministic seed
    /// from std::random_device, but tests can pass a fixed value for
    /// reproducible sequences.
    explicit GaussianNoiseGenerator(double mean, double stddev,
                                    std::mt19937::result_type seed = std::random_device{}())
        : rng_(seed), dist_(mean, std::max(stddev, fallback_min_stddev))
    {
    }

    // Move-only: copying would duplicate rng_/dist_ state rather than share
    // it, so the copy would produce the exact same sequence of samples as
    // the original from that point on, instead of an independent one.
    GaussianNoiseGenerator(GaussianNoiseGenerator const&) = delete;
    GaussianNoiseGenerator& operator=(GaussianNoiseGenerator const&) = delete;
    GaussianNoiseGenerator(GaussianNoiseGenerator&&) = default;
    GaussianNoiseGenerator& operator=(GaussianNoiseGenerator&&) = default;

    /// @brief Draw a sample.
    /// @returns A value drawn from N(mean, stddev^2).
    [[nodiscard]] double sample() { return dist_(rng_); }

private:
    std::mt19937 rng_;                       ///< Mersenne Twister pseudo-random number generator.
    std::normal_distribution<double> dist_;  ///< Gaussian random number distribution.
};

}  // namespace sensor_model