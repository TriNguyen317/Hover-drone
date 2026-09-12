"""Read-only, disarmed optical-flow axis/scale calibration assistant."""
from __future__ import annotations

import argparse
import json
import math
import statistics
import time
from pathlib import Path

from flight_controller.config import load_config
from flight_controller.inav import INAVClient
from flight_controller.msp import MSPError, MSPTransport


def collect(vehicle, duration_s, hz):
    started = time.monotonic()
    previous = started
    x = y = 0.0
    qualities = []
    samples = 0
    local_forward = local_right = 0.0
    while time.monotonic() - started < duration_s:
        tick = time.monotonic()
        if "ARM" in vehicle.active_modes():
            raise MSPError("FC became armed; calibration stopped")
        status = vehicle.sensor_status()
        if status.optical_flow != 1 or status.rangefinder != 1:
            raise MSPError("Optical flow/rangefinder is not healthy")
        quality, fx, fy, bx, by = vehicle.optical_flow()
        distance = vehicle.rangefinder_m()
        pose = vehicle.local_pose()
        now = time.monotonic()
        dt = now - previous
        previous = now
        if not math.isfinite(distance) or distance <= 0:
            raise MSPError("Invalid rangefinder distance")
        x += math.radians(fx - bx) * distance * dt
        y += math.radians(fy - by) * distance * dt
        yaw = math.radians(pose.yaw_deg)
        local_forward += (pose.velocity_north_mps * math.cos(yaw)
                          + pose.velocity_east_mps * math.sin(yaw)) * dt
        local_right += (-pose.velocity_north_mps * math.sin(yaw)
                        + pose.velocity_east_mps * math.cos(yaw)) * dt
        qualities.append(quality)
        samples += 1
        time.sleep(max(0, 1 / hz - (time.monotonic() - tick)))
    return {"x": x, "y": y, "quality": statistics.mean(qualities), "samples": samples,
            "local_forward": local_forward, "local_right": local_right}


def dominant(result):
    axis = "x" if abs(result["x"]) >= abs(result["y"]) else "y"
    other = "y" if axis == "x" else "x"
    magnitude = abs(result[axis])
    separation = magnitude / max(abs(result[other]), 1e-6)
    sign = 1 if result[axis] > 0 else -1
    return axis, sign, magnitude, separation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path("config.json"))
    parser.add_argument("--distance-m", type=float, default=0.5)
    parser.add_argument("--stage-seconds", type=float, default=4.0)
    parser.add_argument("--hz", type=float, default=5.0)
    parser.add_argument("--output", type=Path, default=Path("optical_flow_calibration.json"))
    parser.add_argument("--props-removed", action="store_true")
    args = parser.parse_args()
    if not args.props_removed:
        parser.error("Remove every propeller, then pass --props-removed")
    if not 0.2 <= args.distance_m <= 2 or not 2 <= args.stage_seconds <= 10 or not 2 <= args.hz <= 10:
        parser.error("distance 0.2..2 m, stage 2..10 s, hz 2..10")
    config = load_config(args.config)
    try:
        with MSPTransport(config.serial.port, config.serial.baud,
                          request_timeout=config.serial.request_timeout_s,
                          retries=config.serial.retries) as transport:
            vehicle = INAVClient(transport)
            if "ARM" in vehicle.active_modes():
                raise MSPError("FC must be DISARMED")
            print("Keep the drone level, at constant sensor-to-floor height.")
            input(f"Press Enter, then translate exactly {args.distance_m:.2f} m FORWARD during the measurement...")
            forward = collect(vehicle, args.stage_seconds, args.hz)
            input(f"Reposition it. Press Enter, then translate exactly {args.distance_m:.2f} m RIGHT...")
            right = collect(vehicle, args.stage_seconds, args.hz)
            fa, fs, fm, fsep = dominant(forward)
            ra, rs, rm, rsep = dominant(right)
            if fa == ra or min(fm, rm) < 0.03 or min(fsep, rsep) < 2:
                raise MSPError("Axis result is ambiguous; improve texture/light and repeat slower")
            if min(forward["quality"], right["quality"]) < config.optical_flow.min_quality:
                raise MSPError("Mean flow quality is below configured threshold")
            scale = statistics.median((args.distance_m / fm, args.distance_m / rm))
            if not 0.1 <= scale <= 10:
                raise MSPError(f"Calculated scale {scale:.3f} is implausible")
            local_pose_available = max(
                abs(forward["local_forward"]), abs(forward["local_right"]),
                abs(right["local_forward"]), abs(right["local_right"]),
            ) >= 0.03
            local_pose_ok = (
                local_pose_available
                and forward["local_forward"] > 0
                and right["local_right"] > 0
                and abs(forward["local_forward"]) > 2 * abs(forward["local_right"])
                and abs(right["local_right"]) > 2 * abs(right["local_forward"])
            )
            result = {
                "calibrated": False,
                "forward_axis": fa, "right_axis": ra,
                "forward_sign": fs, "right_sign": rs,
                "scale": round(scale, 5),
                "forward_axis_separation": round(fsep, 2),
                "right_axis_separation": round(rsep, 2),
                "forward_mean_quality": round(forward["quality"], 1),
                "right_mean_quality": round(right["quality"], 1),
                "inav_local_pose_available_disarmed": local_pose_available,
                "inav_local_pose_direction_ok": local_pose_ok,
                "inav_forward_test_body_m": {
                    "forward": round(forward["local_forward"], 4),
                    "right": round(forward["local_right"], 4),
                },
                "inav_right_test_body_m": {
                    "forward": round(right["local_forward"], 4),
                    "right": round(right["local_right"], 4),
                },
                "note": "Do not set calibrated=true unless repeated raw results agree and INAV local-pose direction is verified"
            }
            args.output.parent.mkdir(parents=True, exist_ok=True)
            with args.output.open("x", encoding="utf-8") as handle:
                handle.write(json.dumps(result, indent=2) + "\n")
            print(json.dumps(result, indent=2))
            print(f"Saved recommendation only: {args.output}; config.json was NOT changed.")
    except (MSPError, OSError, ValueError) as exc:
        print(f"[ERROR] {exc}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
