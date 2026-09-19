#pragma once

#include "simulation/uav_3dof_model.hpp"
#include "target/concepts.hpp"

#include <fstream>
#include <iomanip>
#include <string>
#include <string_view>
#include <vector>

/// @brief One time-stamped sample of the interceptor and target states.
struct TrajectorySample
{
    double time_s{0.0};
    simulation::UAV3DofModel::StateVec interceptor_state;
    target::TargetState target_state;
};

/// @brief Write a run of interceptor/target samples to CSV.
/// @details Columns are time_s, int_{x,y,z,v,psi,gamma} and
/// tgt_{x,y,z,vx,vy,vz}, one row per sample.
/// @param[in] path Output file path.
/// @param[in] samples The samples to write, in time order.
inline void writeTrajectoryCsv(std::string_view path, std::vector<TrajectorySample> const& samples)
{
    std::ofstream out{std::string{path}};
    out << std::setprecision(10);

    out << "time_s,int_x_m,int_y_m,int_z_m,int_v_mps,int_psi_rad,int_gamma_rad,"
           "tgt_x_m,tgt_y_m,tgt_z_m,tgt_vx_mps,tgt_vy_mps,tgt_vz_mps\n";

    for (auto const& sample : samples)
    {
        auto const& x = sample.interceptor_state;
        auto const& tgt = sample.target_state;
        out << sample.time_s << ',' << x[0] << ',' << x[1] << ',' << x[2] << ',' << x[3] << ','
            << x[4] << ',' << x[5] << ',' << tgt.position_m.x() << ',' << tgt.position_m.y() << ','
            << tgt.position_m.z() << ',' << tgt.velocity_mps.x() << ',' << tgt.velocity_mps.y()
            << ',' << tgt.velocity_mps.z() << '\n';
    }
}
