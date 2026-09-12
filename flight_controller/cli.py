from __future__ import annotations

import argparse
import sys
from pathlib import Path

from .config import ConfigError, load_config
from .inav import INAVClient
from .mission import MissionController
from .modes import ManualTakeover, SafetyStop
from .msp import MSPError, MSPTransport
from .simulator import SimulationClock, SimulationVehicle


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Pi-side altitude + GPS position PID mission for INAV"
    )
    parser.add_argument(
        "--config",
        default=str(Path(__file__).resolve().parents[1] / "config.example.json"),
        help="JSON configuration path",
    )
    parser.add_argument(
        "--simulate",
        action="store_true",
        help="Run the complete mission against the deterministic simulator",
    )
    parser.add_argument(
        "--fly",
        action="store_true",
        help="Permit real MSP RC/ARM/flight after interactive confirmation",
    )
    parser.add_argument(
        "--altitude-only",
        action="store_true",
        help=(
            "Auto-arm, take off, hold altitude, land and auto-disarm; "
            "pilot/INAV retain roll, pitch and yaw"
        ),
    )
    parser.add_argument(
        "--bench-arm-test",
        action="store_true",
        help=(
            "Props-off test: verify override, auto-arm at minimum throttle, "
            "log telemetry briefly, then auto-disarm"
        ),
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    config_path = Path(args.config).resolve()
    try:
        config = load_config(config_path)
        altitude_only = args.altitude_only or args.bench_arm_test
        if args.simulate:
            vehicle = SimulationVehicle(config, altitude_only=altitude_only)
            controller = MissionController(
                vehicle,
                config,
                clock=SimulationClock(vehicle),
                fly=True,
                simulation=True,
                altitude_only=altitude_only,
                bench_arm_test=args.bench_arm_test,
                log_root=config_path.parent,
            )
            controller.run()
            return 0

        with MSPTransport(
            config.serial.port,
            config.serial.baud,
            request_timeout=config.serial.request_timeout_s,
            retries=config.serial.retries,
        ) as transport:
            controller = MissionController(
                INAVClient(transport),
                config,
                fly=args.fly,
                altitude_only=altitude_only,
                bench_arm_test=args.bench_arm_test,
                log_root=config_path.parent,
            )
            controller.run()
            return 0
    except ManualTakeover as exc:
        print(f"\n[MANUAL TAKEOVER] {exc}", file=sys.stderr)
        print("MSP stream stopped. Fly/land with the physical transmitter.", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("\n[STOP] MSP stream stopped; switch takeover OFF and fly/land with RC.", file=sys.stderr)
        return 130
    except (ConfigError, SafetyStop, MSPError, OSError, ValueError) as exc:
        print(f"\n[ABORT] {exc}", file=sys.stderr)
        print(
            "If the aircraft is airborne: switch physical takeover OFF immediately "
            "and land using RC/RTH. Software does not disarm in flight after a fault.",
            file=sys.stderr,
        )
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
