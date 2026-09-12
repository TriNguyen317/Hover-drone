"""Offline flight-log plots; no serial/vehicle access."""
from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
import numpy as np


def rotation(roll, pitch, yaw):
    p, t, y = np.radians([roll, pitch, yaw])
    cp, sp, ct, st, cy, sy = np.cos(p), np.sin(p), np.cos(t), np.sin(t), np.cos(y), np.sin(y)
    return np.array([[cy*ct, cy*st*sp-sy*cp, cy*st*cp+sy*sp],
                     [sy*ct, sy*st*sp+cy*cp, sy*st*cp-cy*sp],
                     [-st, ct*sp, ct*cp]])


def export(log: Path, output: Path, animate=False):
    with log.open(newline="", encoding="utf-8-sig") as handle:
        rows = list(csv.DictReader(handle))
    def number(row, key):
        try:
            return float(row.get(key, ""))
        except (ValueError, TypeError):
            return np.nan
    # Exclude event-only rows, preserving missing sensor values as NaN.
    rows = [r for r in rows if np.isfinite(number(r, "time_s"))
            and np.isfinite(number(r, "height_agl_m"))]
    if not rows:
        raise ValueError("No telemetry rows with time_s and height_agl_m")
    def col(key):
        return np.array([number(r, key) for r in rows])
    time = col("time_s")
    relative_mode = any(r.get("position_source") == "FLOW_RELATIVE" for r in rows)
    position_label = "Selected relative estimate" if relative_mode else "GPS displacement from calibrated reference"
    velocity_label = "INAV local-pose / selected" if relative_mode else "Measured GNSS"
    if np.any(np.diff(time) < 0):
        raise ValueError("Log timestamps must be nondecreasing")
    output.mkdir(parents=True, exist_ok=True)
    saved = []
    plt.rcParams.update({"font.size": 10, "axes.grid": True})

    def save(fig, name):
        path = output / name
        fig.savefig(path, dpi=160, bbox_inches="tight")
        saved.append(path)

    def panels(name, specifications):
        fig, axes = plt.subplots(len(specifications), 1, figsize=(11, 3*len(specifications)),
                                 sharex=True, squeeze=False)
        for ax, (title, unit, keys) in zip(axes[:, 0], specifications):
            for key, label in keys:
                values = col(key)
                if np.any(np.isfinite(values)):
                    ax.plot(time, values, label=label, linewidth=1.2)
            ax.set(title=title, ylabel=unit)
            if ax.lines:
                ax.legend(loc="best")
            else:
                ax.text(.5, .5, "No data in this log", transform=ax.transAxes, ha="center")
        axes[-1, 0].set_xlabel("Time [s]")
        fig.suptitle(log.name)
        fig.tight_layout()
        save(fig, name)
        plt.close(fig)

    panels("position.png", [
        ("Altitude relative to calibrated ground", "m", [("height_agl_m", "Measured"), ("height_setpoint_m", "Requested")]),
        (position_label, "m", [("north_m", "Control North"), ("east_m", "Control East"),
          ("gps_raw_north_m", "Raw GPS North"), ("gps_raw_east_m", "Raw GPS East"),
          ("position_error_m", "Control distance")]),
        ("Attitude", "deg", [("roll_deg", "Roll"), ("pitch_deg", "Pitch"), ("yaw_deg", "Yaw")])])
    panels("velocity.png", [
        (f"{axis.capitalize()} velocity", "m/s", [(f"velocity_{axis}_mps", velocity_label),
          (f"velocity_{axis}_setpoint_mps", "Requested")]) for axis in ("north", "east")])
    panels("pid.png", [
        (f"{axis.capitalize()} controller terms", "deg", [(f"gps_{axis}_{term}_deg", term.upper())
          for term in ("p", "i", "d", "output")]) for axis in ("north", "east")]+[
        ("Altitude PID", "RC units", [("alt_p", "P"), ("alt_i", "I"), ("alt_d", "D")])])
    panels("motors.png", [
        ("FC motor commands (not RPM)", "FC units", [(f"motor_{i}_output", f"M{i}") for i in range(1,5)]),
        ("ESC RPM (zeros do not prove motors stopped)", "RPM", [(f"motor_{i}_rpm", f"M{i}") for i in range(1,5)]),
        ("RC commands", "RC units", [("roll_pwm", "Roll"), ("pitch_pwm", "Pitch"), ("throttle_pwm", "Throttle")])])
    panels("gps_quality.png", [
        ("GPS satellites", "count", [("satellites", "Satellites")]),
        ("HDOP", "unitless", [("hdop", "HDOP")]),
        ("GPS position control enabled", "0 / 1", [("position_pid_enabled", "Enabled")])])

    north, east, down = col("north_m"), col("east_m"), -col("height_agl_m")
    valid = np.isfinite(north) & np.isfinite(east) & np.isfinite(down)
    if not valid.any():
        print("No horizontal position estimate: trajectory omitted.")
        return saved
    indices = np.flatnonzero(valid)
    fig, ax = plt.subplots(figsize=(8, 7))
    ax.plot(east, north, alpha=.5)
    points = ax.scatter(east[valid], north[valid], c=time[valid], s=12, cmap="viridis")
    fig.colorbar(points, ax=ax, label="Time [s]")
    ax.scatter([0], [0], marker="*", s=140, color="red", label="Calibrated anchor")
    ax.scatter(east[indices[[0,-1]]], north[indices[[0,-1]]], marker="x", label="First / last sample")
    raw_n, raw_e = col("gps_raw_north_m"), col("gps_raw_east_m")
    raw_valid = np.isfinite(raw_n) & np.isfinite(raw_e) & (col("gps_sample_valid") == 1)
    if relative_mode and raw_valid.any():
        ax.plot(raw_e[raw_valid], raw_n[raw_valid], "--", alpha=.55, label="Raw GPS")
    ax.set(xlabel="East [m]", ylabel="North [m]",
           title="Relative estimated track" if relative_mode else "GPS-measured track (includes GNSS drift)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.legend()
    save(fig, "trajectory_xy.png")
    plt.close(fig)

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(north, east, down,
            label="Relative estimate + height" if relative_mode else "GPS + height measurement",
            color="steelblue")
    ax.plot(np.zeros(len(rows)), np.zeros(len(rows)), -col("height_setpoint_m"),
            "k--", label="Requested hover path")
    ax.scatter([0], [0], [0], marker="*", color="red", s=90, label="Calibrated ground")
    ax.set(xlabel="North [m]", ylabel="East [m]", zlabel="Down [m]",
           title="Estimated trajectory, NED" if relative_mode else "Measured trajectory, NED (GPS drift included)")
    # Equal metric scale; upward flight has negative Down.
    clouds = [np.r_[north[valid], 0], np.r_[east[valid], 0],
              np.r_[down[valid], 0, -col("height_setpoint_m")[np.isfinite(col("height_setpoint_m"))]]]
    span = max(max(np.ptp(v) for v in clouds), 1.0) * 1.15
    for values, setter in zip(clouds, (ax.set_xlim, ax.set_ylim, ax.set_zlim)):
        mid = (values.min()+values.max())/2
        setter(mid-span/2, mid+span/2)
    ax.invert_zaxis()
    ax.set_box_aspect((1,1,1))
    ax.view_init(elev=25, azim=35)
    ax.legend(loc="upper left")
    save(fig, "trajectory_3d.png")
    if animate:
        attitude = np.column_stack([col(k) for k in ("roll_deg", "pitch_deg", "yaw_deg")])
        body = np.array([[1,1,0], [-1,-1,0], [1,-1,0], [-1,1,0]]).T * .20/np.sqrt(2)
        arms = [ax.plot([], [], [], linewidth=2, marker="o", color=c)[0] for c in ("orange", "green")]
        trail, = ax.plot([], [], [], color="navy", linewidth=2)
        label = ax.text2D(.03, .93, "", transform=ax.transAxes)
        # Uniform replay time, bounded file size; final sample always included.
        frames = np.searchsorted(time, np.linspace(time[0], time[-1], min(300, max(2, int((time[-1]-time[0])*10)+1))))
        def update(i):
            trail.set_data_3d(north[:i+1], east[:i+1], down[:i+1])
            if valid[i] and np.isfinite(attitude[i]).all():
                endpoints = rotation(*attitude[i]) @ body + np.array([north[i], east[i], down[i]])[:,None]
                for arm, pair in zip(arms, ([0,1], [2,3])):
                    arm.set_data_3d(*endpoints[:,pair])
            else:
                for arm in arms:
                    arm.set_data_3d([], [], [])
            label.set_text(f"t={time[i]:.2f}s | {rows[i].get('state','')} | UAV drawing: illustrative")
            return *arms, trail, label
        animation = FuncAnimation(fig, update, frames=frames, blit=False)
        path = output / "trajectory_replay.gif"
        animation.save(path, writer=PillowWriter(fps=10), dpi=85)
        saved.append(path)
    plt.close(fig)
    return saved


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", nargs="?", type=Path, help="mission CSV; defaults to latest logs/mission_*.csv")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--animate", action="store_true", help="Also save bounded 3D replay GIF")
    args = parser.parse_args()
    log = args.log
    if log is None:
        logs = list(Path("logs").glob("mission_*.csv"))
        if not logs:
            parser.error("No mission CSV in logs/")
        log = max(logs, key=lambda p: p.stat().st_mtime)
    for path in export(log, args.output or log.parent / (log.stem+"_plots"), args.animate):
        print(path)


if __name__ == "__main__":
    main()
