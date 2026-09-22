#pragma once

#include <ilqr/core/types.hpp>

namespace guidance
{

/// The iLQR problem dimensions shared by the augmented dynamics and the terminal cost.
using Dims = ilqr::Dims<7, 3, double>;

/// @brief Tuning for the log-sum-exp softmin term
struct SoftminConfig
{
    double d_scale;  ///< Distance normalization for q_k.
    double min_q;    ///< Numerical-stability shift for avoiding underflow of exp()
    double beta;     ///< Softmin sharpness.
};

}  // namespace guidance
