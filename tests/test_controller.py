from __future__ import annotations

import json
import math
import statistics
import struct
import sys
import tempfile
import unittest
import csv
from pathlib import Path

CONTROLLER_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CONTROLLER_ROOT))

from flight_controller.config import AppConfig, ConfigError, PIDConfig, load_config
from flight_controller.geo import circular_mean_deg, local_ne_m, ne_to_body
from flight_controller.inav import (
    INAVClient,
    MSP2_INAV_ESC_RPM,
    MSP_ALTITUDE,
    MSP_ATTITUDE,
    MSP_MOTOR,
    MSP_RAW_GPS,
)
from flight_controller.mission import (
    MissionController,
    MissionState,
    format_motor_outputs,
    format_motor_rpm,
)
from flight_controller.models import ModeRange, SensorStatus
from flight_controller.modes import choose_channels, validate_mode_configuration
from flight_controller.msp import MSPError, build_v2_request
from flight_controller.pid import PID
from flight_controller.simulator import SimulationClock, SimulationVehicle
from reset_inav_and_check_rx import enter_rp4td_bind_mode, read_safety_state


class FakeTransport:
    def __init__(self, responses: dict[int, bytes]) -> None:
        self.responses = responses
        self.requests: list[tuple[int, bytes]] = []

    def request(self, command: int, payload: bytes = b"") -> bytes:
        self.requests.append((command, payload))
        return self.responses.get(command, b"")

    def request_v2(self, command: int, payload: bytes = b"", flags: int = 0) -> bytes:
        self.requests.append((command, payload))
        return self.responses.get(command, b"")

    def send(self, command: int, payload: bytes = b"") -> None:
        self.requests.append((command, payload))


class NoGPSVehicle(SimulationVehicle):
    def sensor_status(self) -> SensorStatus:
        return SensorStatus(False, 1, 1, 3, 1, 3, 1, 0, 0)

    def gps(self):
        raise AssertionError("altitude-only mode must not request GPS telemetry")


class StartupFlakyVehicle(SimulationVehicle):
    def __init__(self, config: AppConfig, failures: int) -> None:
        super().__init__(config, altitude_only=True)
        self.failures = failures
        self.api_attempts = 0

    def api_version(self):
        self.api_attempts += 1
        if self.api_attempts <= self.failures:
            raise MSPError("temporary startup timeout")
        return super().api_version()


class ResetSafetyVehicle:
    def __init__(self, channels: list[int], modes: set[str]) -> None:
        self.channels = channels
        self.modes = modes

    def rc_channels(self) -> list[int]:
        return self.channels

    def active_modes(self) -> set[str]:
        return self.modes


class FakeCLISerial:
    def __init__(self) -> None:
        self.writes: list[bytes] = []
        self.buffer = bytearray()

    @property
    def in_waiting(self) -> int:
        return len(self.buffer)

    def reset_input_buffer(self) -> None:
        self.buffer.clear()

    def write(self, data: bytes) -> int:
        self.writes.append(data)
        if data.startswith(b"#"):
            self.buffer.extend(b"Entering CLI Mode\r\n# ")
        elif data.startswith(b"bind_rx"):
            self.buffer.extend(b"bind_rx\r\n# ")
        elif data.startswith(b"exit"):
            self.buffer.extend(b"Leaving CLI mode\r\n")
        return len(data)

    def flush(self) -> None:
        pass

    def read(self, count: int) -> bytes:
        result = bytes(self.buffer[:count])
        del self.buffer[:count]
        return result


class FakeCLITransport:
    def __init__(self) -> None:
        self.serial = FakeCLISerial()


class ControllerTests(unittest.TestCase):
    def test_rp4td_bind_uses_cli_bind_rx_and_exits_without_save(self) -> None:
        transport = FakeCLITransport()
        output = enter_rp4td_bind_mode(transport)  # type: ignore[arg-type]
        self.assertIn("bind_rx", output)
        self.assertEqual(
            transport.serial.writes,
            [b"#\r\n", b"bind_rx\r\n", b"exit\r\n"],
        )

    def test_reset_refuses_armed_fc(self) -> None:
        vehicle = ResetSafetyVehicle([1500] * 7 + [1000], {"ARM"})
        with self.assertRaisesRegex(RuntimeError, "đang ARM"):
            read_safety_state(vehicle, AppConfig())  # type: ignore[arg-type]

    def test_reset_refuses_takeover_on(self) -> None:
        vehicle = ResetSafetyVehicle([1500] * 7 + [1800], set())
        with self.assertRaisesRegex(RuntimeError, "takeover đang ON"):
            read_safety_state(vehicle, AppConfig())  # type: ignore[arg-type]

    def test_motor_rpm_console_format_labels_all_four_motors(self) -> None:
        self.assertEqual(
            format_motor_rpm((5100, 5200, 5300, 5400)),
            "M1=5100 M2=5200 M3=5300 M4=5400",
        )

    def test_motor_output_console_format_labels_all_four_motors(self) -> None:
        self.assertEqual(
            format_motor_outputs((1210, 1220, 1230, 1240)),
            "M1=1210 M2=1220 M3=1230 M4=1240",
        )

    def test_pid_saturates_and_does_not_wind_up(self) -> None:
        pid = PID(PIDConfig(10.0, 5.0, 0.0, 20.0, 100.0))
        for _ in range(100):
            terms = pid.update(10.0, 0.0, 0.1)
        self.assertEqual(terms.output, 20.0)
        self.assertAlmostEqual(pid.integral, 0.0)

    def test_pid_can_hold_integrator_before_liftoff(self) -> None:
        pid = PID(PIDConfig(10.0, 5.0, 0.0, 100.0, 100.0))
        terms = pid.update(1.0, 0.0, 0.5, integrate=False)
        self.assertAlmostEqual(pid.integral, 0.0)
        self.assertAlmostEqual(terms.i, 0.0)
        pid.update(1.0, 0.0, 0.5, integrate=True)
        self.assertGreater(pid.integral, 0.0)

    def test_geo_frames(self) -> None:
        north, east = local_ne_m(10.0, 106.0, 10.0001, 106.0)
        self.assertTrue(10.5 < north < 11.7)
        self.assertAlmostEqual(east, 0.0, places=5)
        forward, right = ne_to_body(10.0, 0.0, 90.0)
        self.assertAlmostEqual(forward, 0.0, places=6)
        self.assertAlmostEqual(right, -10.0, places=6)
        self.assertTrue(circular_mean_deg([359.0, 1.0]) < 1e-6)

    def test_real_modes_do_not_request_typed_confirmation(self) -> None:
        config = load_config(CONTROLLER_ROOT / "config.example.json")
        def reject_input(prompt: str) -> str:
            self.fail("Typed confirmation must not be requested")
        for altitude_only, bench in ((False, False), (True, False), (True, True)):
            with self.subTest(altitude_only=altitude_only, bench=bench):
                messages = []
                controller = MissionController(
                    SimulationVehicle(config), config, fly=True,
                    altitude_only=altitude_only, bench_arm_test=bench,
                    confirmation=reject_input, output=messages.append,
                )
                controller._announce_real_flight()
                self.assertTrue(any("physical CH8" in msg for msg in messages))
                self.assertFalse(controller.armed)

    def test_mode_frame_keeps_physical_takeover_low(self) -> None:
        ranges = [
            ModeRange(0, "ARM", 0, 32, 48),
            ModeRange(1, "ANGLE", 1, 32, 48),
            ModeRange(50, "MSP RC OVERRIDE", 3, 32, 48),
        ]
        validate_mode_configuration(ranges, 7, 1700)
        frame = choose_channels(
            ranges,
            {"ARM", "ANGLE"},
            12,
            7,
            throttle=1000,
        )
        self.assertTrue(ranges[0].contains(frame[4]))
        self.assertTrue(ranges[1].contains(frame[5]))
        self.assertEqual(frame[7], 1000)

    def test_inav_parsers(self) -> None:
        responses = {
            MSP_RAW_GPS: struct.pack(
                "<BBiiHHHH", 2, 12, 107622403, 1066812774, 15, 42, 900, 110
            ),
            MSP_ALTITUDE: struct.pack("<ihi", 125, -20, 10_125),
            MSP_ATTITUDE: struct.pack("<hhH", 15, -25, 359),
            MSP_MOTOR: struct.pack("<8H", 1210, 1220, 1230, 1240, 0, 0, 0, 0),
            MSP2_INAV_ESC_RPM: struct.pack("<4I", 5100, 5200, 5300, 5400),
        }
        client = INAVClient(FakeTransport(responses))  # type: ignore[arg-type]
        self.assertAlmostEqual(client.gps().lat, 10.7622403)
        self.assertAlmostEqual(client.gps().ground_speed_mps, 0.42)
        self.assertAlmostEqual(client.altitude().vertical_speed_mps, -0.2)
        self.assertEqual(client.attitude().yaw_deg, 359.0)
        self.assertEqual(client.motor_outputs()[:4], [1210, 1220, 1230, 1240])
        self.assertEqual(client.motor_rpm(), [5100, 5200, 5300, 5400])

    def test_msp_v2_esc_rpm_request(self) -> None:
        self.assertEqual(
            build_v2_request(MSP2_INAV_ESC_RPM),
            bytes.fromhex("24 58 3c 00 40 20 00 00 f5"),
        )

    def test_set_raw_rc_is_write_only(self) -> None:
        transport = FakeTransport({})
        client = INAVClient(transport)  # type: ignore[arg-type]
        client.set_raw_rc([1500, 1500, 1500, 1000, 2000, 2000, 1000, 1000], [0, 1, 2, 3])
        self.assertEqual(len(transport.requests), 1)
        command, payload = transport.requests[0]
        self.assertEqual(command, 200)
        self.assertEqual(len(payload), 16)

    def test_preflight_retries_initial_msp_handshake(self) -> None:
        config = load_config(CONTROLLER_ROOT / "config.example.json")
        vehicle = StartupFlakyVehicle(config, failures=3)
        controller = MissionController(
            vehicle,
            config,
            clock=SimulationClock(vehicle),
            altitude_only=True,
            output=lambda _line: None,
        )
        controller.preflight()
        self.assertEqual(vehicle.api_attempts, 4)

    def test_invalid_height_envelope_is_rejected(self) -> None:
        data = json.loads((CONTROLLER_ROOT / "config.example.json").read_text(encoding="utf-8"))
        data["control"]["target_altitude_m"] = 3.5
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "bad.json"
            path.write_text(json.dumps(data), encoding="utf-8")
            with self.assertRaises(ConfigError):
                load_config(path)

    def test_complete_simulated_mission_with_wind(self) -> None:
        config = load_config(CONTROLLER_ROOT / "config.example.json")
        vehicle = SimulationVehicle(config)
        with tempfile.TemporaryDirectory() as temp:
            controller = MissionController(
                vehicle,
                config,
                clock=SimulationClock(vehicle),
                fly=True,
                simulation=True,
                output=lambda _line: None,
                log_root=temp,
            )
            log_path = controller.run()
            self.assertEqual(controller.state, MissionState.COMPLETE)
            self.assertFalse(controller.armed)
            self.assertIsNotNone(log_path)
            assert log_path is not None
            self.assertTrue(log_path.exists())
            calibration_paths = list(Path(temp).glob("logs/calibration_*.csv"))
            self.assertEqual(len(calibration_paths), 1)
            with calibration_paths[0].open(newline="", encoding="utf-8") as handle:
                calibration_rows = list(csv.DictReader(handle))
            samples = [
                row for row in calibration_rows if row["record_type"] == "SAMPLE"
            ]
            references = [
                row for row in calibration_rows if row["record_type"] == "REFERENCE"
            ]
            self.assertEqual(len(samples), config.sensors.calibration_samples)
            self.assertEqual(len(references), 1)
            reference = references[0]
            self.assertEqual(reference["roll_method"], "mean")
            self.assertEqual(reference["pitch_method"], "mean")
            self.assertEqual(reference["yaw_method"], "circular_mean")
            self.assertEqual(reference["height_method"], "median")
            self.assertEqual(reference["gps_method"], "median")
            self.assertAlmostEqual(
                float(reference["gps_lat"]),
                statistics.median(float(row["gps_lat"]) for row in samples),
                places=12,
            )
            self.assertAlmostEqual(
                float(reference["gps_lon"]),
                statistics.median(float(row["gps_lon"]) for row in samples),
                places=12,
            )
            self.assertAlmostEqual(
                float(reference["height_raw_m"]),
                statistics.median(float(row["height_raw_m"]) for row in samples),
                places=12,
            )
            with log_path.open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
            states = [row["state"] for row in rows]
            self.assertIn(MissionState.ALTITUDE_HOVER.value, states)
            self.assertIn(MissionState.GPS_ACQUIRE.value, states)
            self.assertIn(MissionState.GPS_HOLD.value, states)
            self.assertIn(MissionState.LAND.value, states)
            self.assertLess(
                states.index(MissionState.ALTITUDE_HOVER.value),
                states.index(MissionState.GPS_ACQUIRE.value),
            )
            initial_hover = [
                row for row in rows if row["state"] == MissionState.ALTITUDE_HOVER.value
            ]
            self.assertTrue(initial_hover)
            self.assertTrue(all(row["position_pid_enabled"] == "1" for row in initial_hover))
            takeoff_rows = [
                row for row in rows if row["state"] == MissionState.TAKEOFF.value
            ]
            self.assertTrue(any(row["position_pid_enabled"] == "0" for row in takeoff_rows))
            self.assertTrue(any(row["position_pid_enabled"] == "1" for row in takeoff_rows))
            hover_duration = float(initial_hover[-1]["time_s"]) - float(
                initial_hover[0]["time_s"]
            )
            self.assertGreaterEqual(
                hover_duration,
                config.control.altitude_hover_time_s - 0.2,
            )
            gps_rows = [
                row
                for row in rows
                if row["state"]
                in {MissionState.GPS_ACQUIRE.value, MissionState.GPS_HOLD.value}
            ]
            self.assertTrue(all(row["position_pid_enabled"] == "1" for row in gps_rows))
            gps_pid_fields = (
                "gps_north_error_m",
                "gps_north_p_deg",
                "gps_north_i_deg",
                "gps_north_d_deg",
                "gps_north_output_deg",
                "gps_east_error_m",
                "gps_east_p_deg",
                "gps_east_i_deg",
                "gps_east_d_deg",
                "gps_east_output_deg",
                "tilt_north_limited_deg",
                "tilt_east_limited_deg",
                "forward_tilt_deg",
                "right_tilt_deg",
            )
            for key in gps_pid_fields:
                self.assertTrue(all(row[key] != "" for row in gps_rows), key)
            for row in gps_rows:
                self.assertEqual(row["horizontal_control_mode"], "POSITION_P_VELOCITY_PI")
                vn_set = float(row["velocity_north_setpoint_mps"])
                ve_set = float(row["velocity_east_setpoint_mps"])
                self.assertLessEqual(math.hypot(vn_set, ve_set),
                                     config.control.horizontal_speed_limit_mps + 1e-9)
                self.assertLessEqual(vn_set * float(row["north_m"]) +
                                     ve_set * float(row["east_m"]), 1e-9)
                self.assertAlmostEqual(float(row["gps_north_p_deg"]),
                    config.control.velocity_tilt_kp *
                    (vn_set - float(row["velocity_north_mps"])))
                self.assertEqual(float(row["gps_north_d_deg"]), 0.0)
                self.assertAlmostEqual(
                    float(row["gps_north_error_m"]), -float(row["north_m"]), places=9
                )
                self.assertAlmostEqual(
                    float(row["gps_east_error_m"]), -float(row["east_m"]), places=9
                )
                limited_magnitude = math.hypot(
                    float(row["tilt_north_limited_deg"]),
                    float(row["tilt_east_limited_deg"]),
                )
                self.assertLessEqual(
                    limited_magnitude, config.control.max_tilt_deg + 1e-9
                )
            for key in (
                "gps_calib_lat",
                "gps_calib_lon",
                "gps_measurement_lat",
                "gps_measurement_lon",
                "gps_lat",
                "gps_lon",
                "height_agl_m",
                "height_raw_m",
                "inav_vertical_speed_mps",
                "motor_1_rpm",
                "motor_2_rpm",
                "motor_3_rpm",
                "motor_4_rpm",
            ):
                self.assertTrue(all(row[key] != "" for row in rows), key)
            self.assertEqual(len({row["gps_calib_lat"] for row in rows}), 1)
            self.assertEqual(len({row["gps_calib_lon"] for row in rows}), 1)
            self.assertTrue(
                all(row["gps_measurement_lat"] == row["gps_lat"] for row in rows)
            )
            self.assertTrue(
                all(row["gps_measurement_lon"] == row["gps_lon"] for row in rows)
            )
            airborne = [row for row in rows if float(row["height_agl_m"]) > 0.3]
            self.assertTrue(any(int(row["motor_1_rpm"]) > 0 for row in airborne))
            # Wind is nonzero, so the horizontal loop must issue a correction.
            self.assertTrue(abs(vehicle.north_m - 0.8) > 0.01)

    def test_altitude_only_auto_arm_hover_land_and_disarm(self) -> None:
        config = load_config(CONTROLLER_ROOT / "config.example.json")
        vehicle = SimulationVehicle(config, altitude_only=True)
        with tempfile.TemporaryDirectory() as temp:
            controller = MissionController(
                vehicle,
                config,
                clock=SimulationClock(vehicle),
                fly=True,
                simulation=True,
                altitude_only=True,
                output=lambda _line: None,
                log_root=temp,
            )
            log_path = controller.run()
            self.assertEqual(controller.state, MissionState.COMPLETE)
            self.assertFalse(controller.armed)
            assert log_path is not None
            with log_path.open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
            states = {row["state"] for row in rows}
            self.assertIn(MissionState.TAKEOFF.value, states)
            self.assertIn(MissionState.ALTITUDE_HOVER.value, states)
            self.assertIn(MissionState.LAND.value, states)
            self.assertNotIn(MissionState.GPS_ACQUIRE.value, states)
            self.assertNotIn(MissionState.GPS_HOLD.value, states)
            self.assertTrue(all(row["mission_mode"] == "ALTITUDE_ONLY" for row in rows))
            self.assertTrue(all(row["position_pid_enabled"] == "0" for row in rows))
            self.assertTrue(all(row["gps_north_p_deg"] == "" for row in rows))
            self.assertTrue(all(row["gps_east_p_deg"] == "" for row in rows))
            self.assertTrue(all(row["forward_tilt_deg"] == "" for row in rows))
            self.assertTrue(all(row["gps_calib_lat"] == "" for row in rows))
            self.assertTrue(all(row["gps_measurement_lat"] == "" for row in rows))

    def test_altitude_only_rejects_full_override_mask(self) -> None:
        config = load_config(CONTROLLER_ROOT / "config.example.json")
        vehicle = SimulationVehicle(config)
        controller = MissionController(
            vehicle,
            config,
            clock=SimulationClock(vehicle),
            fly=True,
            simulation=True,
            altitude_only=True,
            output=lambda _line: None,
        )
        with self.assertRaisesRegex(Exception, "Roll/pitch/yaw are still overridden"):
            controller.run()

    def test_full_gps_rejects_altitude_only_override_mask(self) -> None:
        config = load_config(CONTROLLER_ROOT / "config.example.json")
        vehicle = SimulationVehicle(config, altitude_only=True)
        controller = MissionController(
            vehicle,
            config,
            clock=SimulationClock(vehicle),
            fly=True,
            simulation=True,
            output=lambda _line: None,
        )
        with self.assertRaisesRegex(Exception, "msp_override_channels = 63"):
            controller.run()

    def test_props_off_bench_auto_arm_and_disarm(self) -> None:
        config = load_config(CONTROLLER_ROOT / "config.example.json")
        vehicle = SimulationVehicle(config, altitude_only=True)
        with tempfile.TemporaryDirectory() as temp:
            controller = MissionController(
                vehicle,
                config,
                clock=SimulationClock(vehicle),
                fly=True,
                simulation=True,
                altitude_only=True,
                bench_arm_test=True,
                output=lambda _line: None,
                log_root=temp,
            )
            log_path = controller.run()
            self.assertEqual(controller.state, MissionState.COMPLETE)
            self.assertFalse(controller.armed)
            self.assertEqual(vehicle.height_m, 0.0)
            assert log_path is not None
            with log_path.open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
            self.assertTrue(rows)
            self.assertTrue(all(row["state"] == MissionState.BENCH_ARM_TEST.value for row in rows))
            self.assertTrue(all(row["mission_mode"] == "BENCH_ARM_TEST" for row in rows))
            self.assertTrue(
                all(int(row["throttle_pwm"]) == config.control.rc_min for row in rows)
            )

    def test_altitude_only_runs_without_gps_or_compass(self) -> None:
        config = load_config(CONTROLLER_ROOT / "config.example.json")
        vehicle = NoGPSVehicle(config, altitude_only=True)
        with tempfile.TemporaryDirectory() as temp:
            controller = MissionController(
                vehicle,
                config,
                clock=SimulationClock(vehicle),
                fly=True,
                simulation=True,
                altitude_only=True,
                bench_arm_test=True,
                output=lambda _line: None,
                log_root=temp,
            )
            log_path = controller.run()
            self.assertEqual(controller.state, MissionState.COMPLETE)
            assert log_path is not None
            with log_path.open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
            self.assertTrue(rows)
            for field in ("gps_lat", "gps_lon", "gps_fix", "satellites", "hdop"):
                self.assertTrue(all(row[field] == "" for row in rows), field)


if __name__ == "__main__":
    unittest.main()
