#pragma once

#include "math/angles.hpp"
#include "math/state_types.hpp"
#include "sensor_model/radar_model.hpp"

#include <Eigen/Dense>

#include <fstream>
#include <iomanip>
#include <string>
#include <string_view>
#include <vector>

/// @brief One time-stamped sample of a run: the target and interceptor
/// ground-truth states, the EKF's target state estimate, the radar
/// measurement taken of the target, and the control commanded from the
/// interceptor state.
struct TrajectorySample
{
    double time_s{0.0};
    math::CartesianState target_state;
    math::CartesianState target_state_estimate;
    math::VehicleState interceptor_state;
    sensor_model::SensorMeasurement measurement;
    Eigen::Vector3d control{Eigen::Vector3d::Zero()};  ///< [thrust_n, load_factor, bank_angle_rad].
};

/// @brief Write a run's samples to a single CSV.
/// @details Columns are time_s, tgt_{x,y,z,vx,vy,vz,ax,ay,az}_m(ps)(2),
/// est_{x,y,z,vx,vy,vz,ax,ay,az}_m(ps)(2) (the EKF's target state estimate),
/// int_{x,y,z,vx,vy,vz}_m(ps) and int_{heading,fpa,bank}_rad (the
/// interceptor's orientation quaternion, converted to Euler angles here so
/// callers never have to), meas_{timestamp_s,range_m,range_rate_mps,
/// azimuth_rad,elevation_rad}, and cmd_{thrust_n,load_factor,bank_angle_rad},
/// one row per sample.
/// @param[in] path Output file path.
/// @param[in] samples The samples to write, in time order.
inline void writeTrajectoryCsv(std::string_view path, std::vector<TrajectorySample> const& samples)
{
    std::ofstream out{std::string{path}};
    out << std::setprecision(10);

    out << "time_s,"
           "tgt_x_m,tgt_y_m,tgt_z_m,tgt_vx_mps,tgt_vy_mps,tgt_vz_mps,tgt_ax_mps2,tgt_ay_mps2,"
           "tgt_az_mps2,"
           "est_x_m,est_y_m,est_z_m,est_vx_mps,est_vy_mps,est_vz_mps,est_ax_mps2,est_ay_mps2,"
           "est_az_mps2,"
           "int_x_m,int_y_m,int_z_m,int_vx_mps,int_vy_mps,int_vz_mps,"
           "int_heading_rad,int_fpa_rad,int_bank_rad,"
           "meas_timestamp_s,meas_range_m,meas_range_rate_mps,meas_azimuth_rad,meas_elevation_rad,"
           "cmd_thrust_n,cmd_load_factor,cmd_bank_angle_rad\n";

    for (auto const& sample : samples)
    {
        auto const& tgt = sample.target_state;
        auto const& est = sample.target_state_estimate;
        auto const& interceptor = sample.interceptor_state.cartesian;
        auto const euler = math::eulerAnglesFromAttitude(sample.interceptor_state.attitude);
        auto const& meas = sample.measurement;
        auto const& u = sample.control;

        out << sample.time_s << ',' << tgt.position_m.x() << ',' << tgt.position_m.y() << ','
            << tgt.position_m.z() << ',' << tgt.velocity_mps.x() << ',' << tgt.velocity_mps.y()
            << ',' << tgt.velocity_mps.z() << ',' << tgt.acceleration_mps2.x() << ','
            << tgt.acceleration_mps2.y() << ',' << tgt.acceleration_mps2.z() << ','
            << est.position_m.x() << ',' << est.position_m.y() << ',' << est.position_m.z() << ','
            << est.velocity_mps.x() << ',' << est.velocity_mps.y() << ',' << est.velocity_mps.z()
            << ',' << est.acceleration_mps2.x() << ',' << est.acceleration_mps2.y() << ','
            << est.acceleration_mps2.z() << ',' << interceptor.position_m.x() << ','
            << interceptor.position_m.y() << ',' << interceptor.position_m.z() << ','
            << interceptor.velocity_mps.x() << ',' << interceptor.velocity_mps.y() << ','
            << interceptor.velocity_mps.z() << ',' << euler.heading_rad << ','
            << euler.flight_path_angle_rad << ',' << euler.bank_rad << ',' << meas.timestamp.count()
            << ',' << meas.range_m << ',' << meas.range_rate_mps << ',' << meas.azimuth_rad << ','
            << meas.elevation_rad << ',' << u[0] << ',' << u[1] << ',' << u[2] << '\n';
    }
}
