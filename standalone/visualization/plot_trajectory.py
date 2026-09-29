#!/usr/bin/env python3
"""Plot a standalone run's CSV as four separate figures.

Reads the columns produced by standalone/csv_logger.hpp
(time_s, tgt_{x,y,z,vx,vy,vz,ax,ay,az}_m(ps)(2),
est_{x,y,z,vx,vy,vz,ax,ay,az}_m(ps)(2), int_{x,y,z,vx,vy,vz}_m(ps),
int_{heading,fpa,bank}_rad, meas_{timestamp_s,range_m,range_rate_mps,
azimuth_rad,elevation_rad}, cmd_{thrust_n,load_factor,bank_angle_rad}) and
renders:

1. Trajectories: 3D view, top-down (bird's-eye) view and altitude vs time,
   side by side.
2. EKF performance: target ground truth vs estimate, for 3D/planar/altitude
   position and velocity/acceleration magnitude.
3. Interceptor vehicle states (speed, heading, flight-path angle) and
   commanded control inputs (thrust, load factor, bank angle).
4. Radar measurements (range, range rate, azimuth, elevation).

Usage:
    python3 plot_trajectory.py [csv_path] [-o output_stem] [--no-show]
"""

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

INTERCEPTOR_COLOR = "#1f77b4"
TARGET_COLOR = "#d62728"
MEASURED_COLOR = "#7f7f7f"
ESTIMATE_ALPHA = 0.55  # estimate traces reuse their ground-truth color at this alpha


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "csv_path",
        nargs="?",
        default="standalone_trajectory.csv",
        help="path to the trajectory CSV (default: %(default)s)",
    )
    parser.add_argument(
        "-o",
        "--out",
        default=None,
        help="output path stem; writes <stem>_trajectories.png, <stem>_vehicle.png "
        "and <stem>_measurements.png (default: <csv_path> without its extension)",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="save the figures without opening interactive windows",
    )
    return parser.parse_args()


def _point_coords(row: pd.Series, prefix: str, *, is_3d: bool) -> tuple:
    """(x, y[, z]) for one row's prefix (e.g. "int" or "tgt"), each a 1-item list."""
    coords = [[row[f"{prefix}_x_m"]], [row[f"{prefix}_y_m"]]]
    if is_3d:
        coords.append([row[f"{prefix}_z_m"]])
    return tuple(coords)


def _mark_start_end(ax, df: pd.DataFrame, series: list, *, is_3d: bool) -> None:
    """Mark each series' start (circle) and end (X) point.

    series is a list of (column_prefix, color, alpha) triples, e.g.
    [("tgt", TARGET_COLOR, 1.0)].
    """
    for prefix, color, alpha in series:
        ax.scatter(*_point_coords(df.iloc[0], prefix, is_3d=is_3d), color=color, alpha=alpha,
                   marker="o", s=40, zorder=5)
        ax.scatter(*_point_coords(df.iloc[-1], prefix, is_3d=is_3d), color=color, alpha=alpha,
                   marker="x", s=50, zorder=5)


def plot_trajectories(df: pd.DataFrame) -> plt.Figure:
    """3D trajectory, top-down (bird's-eye) view and altitude vs time."""
    fig = plt.figure(figsize=(15, 5))

    ax_3d = fig.add_subplot(1, 3, 1, projection="3d")
    ax_3d.plot(df["int_x_m"], df["int_y_m"], df["int_z_m"], color=INTERCEPTOR_COLOR,
              label="Interceptor")
    ax_3d.plot(df["tgt_x_m"], df["tgt_y_m"], df["tgt_z_m"], color=TARGET_COLOR, label="Target")
    _mark_start_end(ax_3d, df, [("int", INTERCEPTOR_COLOR, 1.0), ("tgt", TARGET_COLOR, 1.0)],
                    is_3d=True)
    ax_3d.set_xlabel("x [m]")
    ax_3d.set_ylabel("y [m]")
    ax_3d.set_zlabel("z [m]")
    ax_3d.set_title("3D trajectory")
    ax_3d.legend()

    ax_top = fig.add_subplot(1, 3, 2)
    ax_top.plot(df["int_x_m"], df["int_y_m"], color=INTERCEPTOR_COLOR, label="Interceptor")
    ax_top.plot(df["tgt_x_m"], df["tgt_y_m"], color=TARGET_COLOR, label="Target")
    _mark_start_end(ax_top, df, [("int", INTERCEPTOR_COLOR, 1.0), ("tgt", TARGET_COLOR, 1.0)],
                    is_3d=False)
    ax_top.set_xlabel("x [m]")
    ax_top.set_ylabel("y [m]")
    ax_top.set_title("Top-down (bird's-eye) view")
    ax_top.set_aspect("equal", adjustable="datalim")
    ax_top.grid(True, alpha=0.3)
    ax_top.legend()

    ax_alt = fig.add_subplot(1, 3, 3)
    ax_alt.plot(df["time_s"], df["int_z_m"], color=INTERCEPTOR_COLOR, label="Interceptor")
    ax_alt.plot(df["time_s"], df["tgt_z_m"], color=TARGET_COLOR, label="Target")
    ax_alt.set_xlabel("t [s]")
    ax_alt.set_ylabel("z [m]")
    ax_alt.set_title("Altitude vs time")
    ax_alt.grid(True, alpha=0.3)
    ax_alt.legend()

    fig.suptitle("Target vs interceptor trajectory")
    fig.tight_layout()
    return fig


def plot_ekf_performance(df: pd.DataFrame) -> plt.Figure:
    """Target ground truth vs EKF estimate: position (3D, top-down, altitude)
    and velocity/acceleration magnitude."""
    fig = plt.figure(figsize=(15, 8))
    series = [("tgt", TARGET_COLOR, 1.0), ("est", TARGET_COLOR, ESTIMATE_ALPHA)]

    ax_3d = fig.add_subplot(2, 3, 1, projection="3d")
    ax_3d.plot(df["tgt_x_m"], df["tgt_y_m"], df["tgt_z_m"], color=TARGET_COLOR,
              label="Ground truth")
    ax_3d.plot(df["est_x_m"], df["est_y_m"], df["est_z_m"], color=TARGET_COLOR,
              alpha=ESTIMATE_ALPHA, label="EKF estimate")
    _mark_start_end(ax_3d, df, series, is_3d=True)
    ax_3d.set_xlabel("x [m]")
    ax_3d.set_ylabel("y [m]")
    ax_3d.set_zlabel("z [m]")
    ax_3d.set_title("3D position")
    ax_3d.legend()

    ax_top = fig.add_subplot(2, 3, 2)
    ax_top.plot(df["tgt_x_m"], df["tgt_y_m"], color=TARGET_COLOR, label="Ground truth")
    ax_top.plot(df["est_x_m"], df["est_y_m"], color=TARGET_COLOR, alpha=ESTIMATE_ALPHA,
               label="EKF estimate")
    _mark_start_end(ax_top, df, series, is_3d=False)
    ax_top.set_xlabel("x [m]")
    ax_top.set_ylabel("y [m]")
    ax_top.set_title("Planar (x-y) position")
    ax_top.set_aspect("equal", adjustable="datalim")
    ax_top.grid(True, alpha=0.3)
    ax_top.legend()

    ax_alt = fig.add_subplot(2, 3, 3)
    ax_alt.plot(df["time_s"], df["tgt_z_m"], color=TARGET_COLOR, label="Ground truth")
    ax_alt.plot(df["time_s"], df["est_z_m"], color=TARGET_COLOR, alpha=ESTIMATE_ALPHA,
               label="EKF estimate")
    ax_alt.set_xlabel("t [s]")
    ax_alt.set_ylabel("z [m]")
    ax_alt.set_title("Altitude vs time")
    ax_alt.grid(True, alpha=0.3)
    ax_alt.legend()

    tgt_speed = np.sqrt(df["tgt_vx_mps"] ** 2 + df["tgt_vy_mps"] ** 2 + df["tgt_vz_mps"] ** 2)
    est_speed = np.sqrt(df["est_vx_mps"] ** 2 + df["est_vy_mps"] ** 2 + df["est_vz_mps"] ** 2)

    ax_speed = fig.add_subplot(2, 3, 4)
    ax_speed.plot(df["time_s"], tgt_speed, color=TARGET_COLOR, label="Ground truth")
    ax_speed.plot(df["time_s"], est_speed, color=TARGET_COLOR, alpha=ESTIMATE_ALPHA,
                 label="EKF estimate")
    ax_speed.set_xlabel("t [s]")
    ax_speed.set_ylabel("speed [m/s]")
    ax_speed.set_title("Velocity magnitude")
    ax_speed.grid(True, alpha=0.3)
    ax_speed.legend()

    tgt_accel = np.sqrt(df["tgt_ax_mps2"] ** 2 + df["tgt_ay_mps2"] ** 2 + df["tgt_az_mps2"] ** 2)
    est_accel = np.sqrt(df["est_ax_mps2"] ** 2 + df["est_ay_mps2"] ** 2 + df["est_az_mps2"] ** 2)

    ax_accel = fig.add_subplot(2, 3, 5)
    ax_accel.plot(df["time_s"], tgt_accel, color=TARGET_COLOR, label="Ground truth")
    ax_accel.plot(df["time_s"], est_accel, color=TARGET_COLOR, alpha=ESTIMATE_ALPHA,
                 label="EKF estimate")
    ax_accel.set_xlabel("t [s]")
    ax_accel.set_ylabel("accel [m/s^2]")
    ax_accel.set_title("Acceleration magnitude")
    ax_accel.grid(True, alpha=0.3)
    ax_accel.legend()

    fig.delaxes(fig.add_subplot(2, 3, 6))

    fig.suptitle("Target ground truth vs EKF estimate")
    fig.tight_layout()
    return fig


def plot_vehicle_and_controls(df: pd.DataFrame) -> plt.Figure:
    """Interceptor vehicle states (top row) and commanded control inputs (bottom row)."""
    fig, axes = plt.subplots(2, 3, figsize=(15, 8))
    speed_mps = np.sqrt(df["int_vx_mps"] ** 2 + df["int_vy_mps"] ** 2 + df["int_vz_mps"] ** 2)

    ax_speed = axes[0, 0]
    ax_speed.plot(df["time_s"], speed_mps, color=INTERCEPTOR_COLOR)
    ax_speed.set_xlabel("t [s]")
    ax_speed.set_ylabel("speed [m/s]")
    ax_speed.set_title("Speed")
    ax_speed.grid(True, alpha=0.3)

    ax_heading = axes[0, 1]
    ax_heading.plot(df["time_s"], np.degrees(df["int_heading_rad"]), color=INTERCEPTOR_COLOR)
    ax_heading.set_xlabel("t [s]")
    ax_heading.set_ylabel("heading [deg]")
    ax_heading.set_title("Heading")
    ax_heading.grid(True, alpha=0.3)

    ax_fpa = axes[0, 2]
    ax_fpa.plot(df["time_s"], np.degrees(df["int_fpa_rad"]), color=INTERCEPTOR_COLOR)
    ax_fpa.set_xlabel("t [s]")
    ax_fpa.set_ylabel("flight-path angle [deg]")
    ax_fpa.set_title("Flight-path angle")
    ax_fpa.grid(True, alpha=0.3)

    ax_thrust = axes[1, 0]
    ax_thrust.plot(df["time_s"], df["cmd_thrust_n"], color=INTERCEPTOR_COLOR)
    ax_thrust.set_xlabel("t [s]")
    ax_thrust.set_ylabel("thrust [N]")
    ax_thrust.set_title("Commanded thrust")
    ax_thrust.grid(True, alpha=0.3)

    ax_load_factor = axes[1, 1]
    ax_load_factor.plot(df["time_s"], df["cmd_load_factor"], color=INTERCEPTOR_COLOR)
    ax_load_factor.set_xlabel("t [s]")
    ax_load_factor.set_ylabel("load factor [g]")
    ax_load_factor.set_title("Commanded load factor")
    ax_load_factor.grid(True, alpha=0.3)

    ax_bank = axes[1, 2]
    ax_bank.plot(df["time_s"], np.degrees(df["int_bank_rad"]), color=INTERCEPTOR_COLOR,
                label="Vehicle state")
    ax_bank.plot(df["time_s"], np.degrees(df["cmd_bank_angle_rad"]), color=INTERCEPTOR_COLOR,
                linestyle="--", alpha=0.6, label="Commanded")
    ax_bank.set_xlabel("t [s]")
    ax_bank.set_ylabel("bank angle [deg]")
    ax_bank.set_title("Bank angle")
    ax_bank.grid(True, alpha=0.3)
    ax_bank.legend()

    fig.suptitle("Interceptor vehicle states and control inputs")
    fig.tight_layout()
    return fig


def plot_measurements(df: pd.DataFrame) -> plt.Figure:
    """Radar measurements: range, range rate, azimuth, elevation."""
    fig, axes = plt.subplots(2, 2, figsize=(11, 8))

    ax_range = axes[0, 0]
    ax_range.plot(df["time_s"], df["meas_range_m"], color=MEASURED_COLOR)
    ax_range.set_xlabel("t [s]")
    ax_range.set_ylabel("range [m]")
    ax_range.set_title("Range")
    ax_range.grid(True, alpha=0.3)

    ax_range_rate = axes[0, 1]
    ax_range_rate.plot(df["time_s"], df["meas_range_rate_mps"], color=MEASURED_COLOR)
    ax_range_rate.set_xlabel("t [s]")
    ax_range_rate.set_ylabel("range rate [m/s]")
    ax_range_rate.set_title("Range rate")
    ax_range_rate.grid(True, alpha=0.3)

    ax_azimuth = axes[1, 0]
    ax_azimuth.plot(df["time_s"], np.degrees(df["meas_azimuth_rad"]), color=MEASURED_COLOR)
    ax_azimuth.set_xlabel("t [s]")
    ax_azimuth.set_ylabel("azimuth [deg]")
    ax_azimuth.set_title("Azimuth")
    ax_azimuth.grid(True, alpha=0.3)

    ax_elevation = axes[1, 1]
    ax_elevation.plot(df["time_s"], np.degrees(df["meas_elevation_rad"]), color=MEASURED_COLOR)
    ax_elevation.set_xlabel("t [s]")
    ax_elevation.set_ylabel("elevation [deg]")
    ax_elevation.set_title("Elevation")
    ax_elevation.grid(True, alpha=0.3)

    fig.suptitle("Radar measurements")
    fig.tight_layout()
    return fig


def main() -> None:
    args = parse_args()
    csv_path = Path(args.csv_path)
    if not csv_path.is_file():
        raise SystemExit(f"error: no such CSV file: {csv_path}")

    df = pd.read_csv(csv_path)

    out_stem = Path(args.out) if args.out else csv_path.with_suffix("")
    figures = {
        "trajectories": plot_trajectories(df),
        "ekf": plot_ekf_performance(df),
        "vehicle": plot_vehicle_and_controls(df),
        "measurements": plot_measurements(df),
    }

    for name, fig in figures.items():
        out_path = out_stem.with_name(f"{out_stem.name}_{name}.png")
        fig.savefig(out_path, dpi=150)
        print(f"wrote {out_path}")

    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
