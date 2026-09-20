#pragma once

#include "math/cartesian_state.hpp"
#include "simulation/uav_3dof_model.hpp"

#include <fstream>
#include <iomanip>
#include <string>
#include <string_view>
#include <vector>

/// @brief One time-stamped sample of the interceptor and target states, and
/// the control commanded from the interceptor state.
struct TrajectorySample
{
    double time_s{0.0};
    simulation::UAV3DofModel::StateVec interceptor_state;
    math::CartesianState target_state;
    simulation::UAV3DofModel::ControlVec control;
};

/// @brief Write a run of interceptor/target samples to CSV.
/// @details Columns are time_s, int_{x,y,z,v,psi,gamma}, tgt_{x,y,z,vx,vy,vz}
/// and cmd_{thrust_n,load_factor,bank_angle_rad}, one row per sample.
/// @param[in] path Output file path.
/// @param[in] samples The samples to write, in time order.
inline void writeTrajectoryCsv(std::string_view path, std::vector<TrajectorySample> const& samples)
{
    std::ofstream out{std::string{path}};
    out << std::setprecision(10);

    out << "time_s,int_x_m,int_y_m,int_z_m,int_v_mps,int_psi_rad,int_gamma_rad,"
           "tgt_x_m,tgt_y_m,tgt_z_m,tgt_vx_mps,tgt_vy_mps,tgt_vz_mps,"
           "cmd_thrust_n,cmd_load_factor,cmd_bank_angle_rad\n";

    for (auto const& sample : samples)
    {
        auto const& x = sample.interceptor_state;
        auto const& tgt = sample.target_state;
        auto const& u = sample.control;
        out << sample.time_s << ',' << x[0] << ',' << x[1] << ',' << x[2] << ',' << x[3] << ','
            << x[4] << ',' << x[5] << ',' << tgt.position_m.x() << ',' << tgt.position_m.y() << ','
            << tgt.position_m.z() << ',' << tgt.velocity_mps.x() << ',' << tgt.velocity_mps.y()
            << ',' << tgt.velocity_mps.z() << ',' << u[0] << ',' << u[1] << ',' << u[2] << '\n';
    }
}
