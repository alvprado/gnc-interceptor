#!/usr/bin/env python3
"""Plot the interceptor and target trajectories from a standalone run's CSV.

Reads the columns produced by standalone/csv_logger.hpp
(time_s, int_{x,y,z,v,psi,gamma}, tgt_{x,y,z,vx,vy,vz},
cmd_{thrust_n,load_factor,bank_angle_rad}) and renders a 3D trajectory view,
a top-down (X-Y) view, an altitude (Z vs t) profile, the interceptor's speed
(alongside the target's, for comparison), heading and flight-path angle, and
the commanded thrust, load factor and bank angle, so a run can be checked
visually rather than by reading numbers.

Usage:
    python3 plot_trajectory.py [csv_path] [-o output.png] [--no-show]
"""

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

INTERCEPTOR_COLOR = "#1f77b4"
TARGET_COLOR = "#d62728"


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
        help="output image path (default: <csv_path> with a .png extension)",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="save the figure without opening an interactive window",
    )
    return parser.parse_args()


def plot_trajectories(df: pd.DataFrame) -> plt.Figure:
    """Build the 9-panel figure: trajectory views, interceptor state, then
    commanded controls."""
    fig = plt.figure(figsize=(14, 13))
    tgt_speed_mps = np.sqrt(df["tgt_vx_mps"] ** 2 + df["tgt_vy_mps"] ** 2 + df["tgt_vz_mps"] ** 2)

    # 3D trajectory.
    ax_3d = fig.add_subplot(3, 3, 1, projection="3d")
    ax_3d.plot(df["int_x_m"], df["int_y_m"], df["int_z_m"], color=INTERCEPTOR_COLOR,
              label="Interceptor")
    ax_3d.plot(df["tgt_x_m"], df["tgt_y_m"], df["tgt_z_m"], color=TARGET_COLOR, label="Target")
    _mark_start_end(ax_3d, df, is_3d=True)
    ax_3d.set_xlabel("x [m]")
    ax_3d.set_ylabel("y [m]")
    ax_3d.set_zlabel("z [m]")
    ax_3d.set_title("3D trajectory")
    ax_3d.legend()

    # Top-down (X-Y) view.
    ax_top = fig.add_subplot(3, 3, 2)
    ax_top.plot(df["int_x_m"], df["int_y_m"], color=INTERCEPTOR_COLOR, label="Interceptor")
    ax_top.plot(df["tgt_x_m"], df["tgt_y_m"], color=TARGET_COLOR, label="Target")
    _mark_start_end(ax_top, df, is_3d=False)
    ax_top.set_xlabel("x [m]")
    ax_top.set_ylabel("y [m]")
    ax_top.set_title("Top-down view")
    ax_top.set_aspect("equal", adjustable="datalim")
    ax_top.grid(True, alpha=0.3)
    ax_top.legend()

    # Altitude profile.
    ax_alt = fig.add_subplot(3, 3, 3)
    ax_alt.plot(df["time_s"], df["int_z_m"], color=INTERCEPTOR_COLOR, label="Interceptor")
    ax_alt.plot(df["time_s"], df["tgt_z_m"], color=TARGET_COLOR, label="Target")
    ax_alt.set_xlabel("t [s]")
    ax_alt.set_ylabel("z [m]")
    ax_alt.set_title("Altitude vs time")
    ax_alt.grid(True, alpha=0.3)
    ax_alt.legend()

    # Speed (interceptor vs target, for comparison).
    ax_speed = fig.add_subplot(3, 3, 4)
    ax_speed.plot(df["time_s"], df["int_v_mps"], color=INTERCEPTOR_COLOR, label="Interceptor")
    ax_speed.plot(df["time_s"], tgt_speed_mps, color=TARGET_COLOR, label="Target")
    ax_speed.set_xlabel("t [s]")
    ax_speed.set_ylabel("speed [m/s]")
    ax_speed.set_title("Speed")
    ax_speed.grid(True, alpha=0.3)
    ax_speed.legend()

    # Heading.
    ax_heading = fig.add_subplot(3, 3, 5)
    ax_heading.plot(df["time_s"], np.degrees(df["int_psi_rad"]), color=INTERCEPTOR_COLOR)
    ax_heading.set_xlabel("t [s]")
    ax_heading.set_ylabel("heading [deg]")
    ax_heading.set_title("Interceptor heading")
    ax_heading.grid(True, alpha=0.3)

    # Flight-path angle.
    ax_gamma = fig.add_subplot(3, 3, 6)
    ax_gamma.plot(df["time_s"], np.degrees(df["int_gamma_rad"]), color=INTERCEPTOR_COLOR)
    ax_gamma.set_xlabel("t [s]")
    ax_gamma.set_ylabel("flight-path angle [deg]")
    ax_gamma.set_title("Interceptor flight-path angle")
    ax_gamma.grid(True, alpha=0.3)

    # Commanded thrust.
    ax_thrust = fig.add_subplot(3, 3, 7)
    ax_thrust.plot(df["time_s"], df["cmd_thrust_n"], color=INTERCEPTOR_COLOR)
    ax_thrust.set_xlabel("t [s]")
    ax_thrust.set_ylabel("thrust [N]")
    ax_thrust.set_title("Commanded thrust")
    ax_thrust.grid(True, alpha=0.3)

    # Commanded load factor.
    ax_load_factor = fig.add_subplot(3, 3, 8)
    ax_load_factor.plot(df["time_s"], df["cmd_load_factor"], color=INTERCEPTOR_COLOR)
    ax_load_factor.set_xlabel("t [s]")
    ax_load_factor.set_ylabel("load factor [g]")
    ax_load_factor.set_title("Commanded load factor")
    ax_load_factor.grid(True, alpha=0.3)

    # Commanded bank angle.
    ax_bank = fig.add_subplot(3, 3, 9)
    ax_bank.plot(df["time_s"], np.degrees(df["cmd_bank_angle_rad"]), color=INTERCEPTOR_COLOR)
    ax_bank.set_xlabel("t [s]")
    ax_bank.set_ylabel("bank angle [deg]")
    ax_bank.set_title("Commanded bank angle")
    ax_bank.grid(True, alpha=0.3)

    fig.tight_layout()
    return fig


def _point_coords(row: pd.Series, prefix: str, *, is_3d: bool) -> tuple:
    """(x, y[, z]) for one row's prefix (e.g. "int" or "tgt"), each a 1-item list."""
    coords = [[row[f"{prefix}_x_m"]], [row[f"{prefix}_y_m"]]]
    if is_3d:
        coords.append([row[f"{prefix}_z_m"]])
    return tuple(coords)


def _mark_start_end(ax, df: pd.DataFrame, *, is_3d: bool) -> None:
    """Mark each trajectory's start (circle) and end (X) point."""
    for prefix, color in (("int", INTERCEPTOR_COLOR), ("tgt", TARGET_COLOR)):
        ax.scatter(*_point_coords(df.iloc[0], prefix, is_3d=is_3d), color=color, marker="o",
                   s=40, zorder=5)
        ax.scatter(*_point_coords(df.iloc[-1], prefix, is_3d=is_3d), color=color, marker="x",
                   s=50, zorder=5)


def main() -> None:
    args = parse_args()
    csv_path = Path(args.csv_path)
    if not csv_path.is_file():
        raise SystemExit(f"error: no such CSV file: {csv_path}")

    df = pd.read_csv(csv_path)

    fig = plot_trajectories(df)

    out_path = Path(args.out) if args.out else csv_path.with_suffix(".png")
    fig.savefig(out_path, dpi=150)
    print(f"wrote {out_path}")

    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
