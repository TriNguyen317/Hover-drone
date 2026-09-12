from __future__ import annotations

import argparse
import time
from pathlib import Path

from flight_controller.config import load_config
from flight_controller.inav import INAVClient
from flight_controller.msp import MSPTransport


def main() -> int:
    parser = argparse.ArgumentParser(description="Read-only INAV GPS monitor")
    parser.add_argument("--config", default="config.json")
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--period", type=float, default=1.0)
    args = parser.parse_args()

    config = load_config(Path(args.config))
    deadline = time.monotonic() + max(1.0, args.duration)
    sample = 0
    with MSPTransport(
        config.serial.port,
        config.serial.baud,
        request_timeout=config.serial.request_timeout_s,
        retries=config.serial.retries,
    ) as transport:
        vehicle = INAVClient(transport)
        while time.monotonic() < deadline:
            started = time.monotonic()
            gps = vehicle.gps()
            sensors = vehicle.sensor_status()
            sample += 1
            fix_name = {0: "NO_FIX", 1: "2D", 2: "3D"}.get(
                gps.fix_type, f"UNKNOWN({gps.fix_type})"
            )
            print(
                f"{sample:03d} fix={fix_name} sats={gps.satellites:02d} "
                f"HDOP={gps.hdop:.2f} lat={gps.lat:.7f} lon={gps.lon:.7f} "
                f"speed={gps.ground_speed_mps:.2f}m/s "
                f"sensor_gps={sensors.gps} overall={int(sensors.overall_healthy)}",
                flush=True,
            )
            time.sleep(max(0.0, args.period - (time.monotonic() - started)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
