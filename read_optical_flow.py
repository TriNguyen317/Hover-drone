"""Read-only INAV 9 optical-flow monitor. No ARM, RC or configuration writes."""
from __future__ import annotations

import argparse
import csv
import struct
import time
from contextlib import ExitStack
from pathlib import Path

from flight_controller.config import load_config
from flight_controller.inav import INAVClient
from flight_controller.msp import MSPError, MSPTransport


FIELDS = ("time_s", "status", "quality", "flow_x_dps", "flow_y_dps",
          "body_x_dps", "body_y_dps")


def decode_flow(payload: bytes) -> tuple[int, int, int, int, int]:
    if len(payload) != 9:
        raise MSPError(f"Invalid optical-flow payload length: {len(payload)} (expected 9)")
    # INAV 9.0.1: quality uint8 + four signed rates, integer degrees/second.
    return struct.unpack("<Bhhhh", payload)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path("config.json"))
    parser.add_argument("--duration", type=float, default=20.0, help="Seconds to read; 0 until Ctrl+C")
    parser.add_argument("--hz", type=float, default=5.0, help="MSP polling frequency (not sensor rate)")
    parser.add_argument("--csv", type=Path, help="New CSV path (existing files are not overwritten)")
    args = parser.parse_args()
    if not 0.5 <= args.hz <= 10 or not 0 <= args.duration < float("inf"):
        parser.error("--hz must be 0.5..10; --duration must be finite and nonnegative")
    config = load_config(args.config)
    try:
        with ExitStack() as stack:
            transport = stack.enter_context(MSPTransport(
                config.serial.port, config.serial.baud,
                request_timeout=config.serial.request_timeout_s,
                retries=config.serial.retries))
            vehicle = INAVClient(transport)
            if "ARM" in vehicle.active_modes():
                raise MSPError("FC is armed; stop flight before starting this diagnostic")
            writer = None
            handle = None
            if args.csv:
                args.csv.parent.mkdir(parents=True, exist_ok=True)
                handle = stack.enter_context(args.csv.open("x", newline="", encoding="utf-8"))
                writer = csv.DictWriter(handle, fieldnames=FIELDS)
                writer.writeheader()
            print("Read-only optical flow. Rates: deg/s; not horizontal speed in m/s.", flush=True)
            print("Status: 0=NONE, 1=OK; other codes are reported unchanged.", flush=True)
            print("INAV exposes integer rates: slow motion may round to zero.", flush=True)
            started = time.monotonic()
            while args.duration == 0 or time.monotonic() - started < args.duration:
                tick = time.monotonic()
                if "ARM" in vehicle.active_modes():
                    raise MSPError("FC became armed; diagnostic stopped")
                status = vehicle.sensor_status().optical_flow
                quality, fx, fy, bx, by = decode_flow(transport.request_v2(0x2001))
                row = dict(zip(FIELDS, (time.monotonic()-started, status, quality, fx, fy, bx, by)))
                print(f"t={row['time_s']:6.2f}s status={status} quality={quality:3d} "
                      f"flow=({fx:+5d},{fy:+5d}) body=({bx:+5d},{by:+5d}) deg/s", flush=True)
                if writer is not None:
                    writer.writerow(row)
                    handle.flush()
                time.sleep(max(0, 1/args.hz - (time.monotonic()-tick)))
    except KeyboardInterrupt:
        print("\nStopped; serial port and log closed.")
    except (MSPError, OSError, ValueError) as exc:
        print(f"[ERROR] {exc}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
