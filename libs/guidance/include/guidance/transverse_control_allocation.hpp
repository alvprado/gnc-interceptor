#pragma once

#include <Eigen/Dense>
#include <numbers>

#include "math/cartesian_state.hpp"

namespace guidance
{

/// @brief Command limits for TransverseControlAllocation.
struct TransverseControlAllocationConfig
{
    double min_load_factor{-3.0};                 ///< Minimum load factor.
    double max_load_factor{9.0};                  ///< Maximum load factor.
    double max_bank_angle_rad{std::numbers::pi};  ///< Maximum |bank angle| in rad.
};

struct TransverseControlAllocationOutput
{
    double load_factor;
    double bank_angle_rad;
};

/// @brief Resolves a commanded acceleration to load factor and bank angle,
/// against the interceptor's velocity direction.
class TransverseControlAllocation
{
public:
    /// @brief Construct a transverse control allocation.
    /// @param[in] config The load factor and bank angle limits.
    explicit TransverseControlAllocation(TransverseControlAllocationConfig const& config) noexcept;

    /// @brief Allocate a commanded acceleration to load factor and bank angle.
    /// @param[in] interceptor The interceptor's state.
    /// @param[in] transverse_acceleration_cmd The commanded transverse acceleration
    /// @returns [load_factor, bank_angle_rad].
    [[nodiscard]] TransverseControlAllocationOutput step(
        math::CartesianState const& interceptor,
        Eigen::Vector3d const& transverse_acceleration_cmd) const noexcept;

private:
    TransverseControlAllocationConfig config_;
};

}  // namespace guidance
