from __future__ import annotations

import csv
import math
import statistics
import time
from dataclasses import asdict
from datetime import datetime
from enum import Enum
from pathlib import Path
from typing import Callable, Protocol, Sequence, TextIO

from .config import AppConfig, PIDConfig
from .geo import circular_mean_deg, local_ne_m, ne_to_body
from .inav import Vehicle
from .models import (
    GPS_FIX_3D, HW_SENSOR_OK, AttitudeData, GPSData, GroundReference, Telemetry,
)
from .msp import MSPError
from .modes import (
    ManualTakeover,
    SafetyStop,
    choose_channels,
    ranges_by_name,
    validate_mode_configuration,
)
from .pid import PID, clamp
from .optical_flow import flow_velocity, FlowWatchdog


class Clock(Protocol):
    def monotonic(self) -> float: ...
    def sleep(self, seconds: float) -> None: ...


class RealClock:
    monotonic = staticmethod(time.monotonic)
    sleep = staticmethod(time.sleep)


def format_motor_rpm(values: Sequence[int]) -> str:
    return " ".join(
        f"M{index}={'n/a' if value < 0 else value}"
        for index, value in enumerate(values, start=1)
    )


def format_motor_outputs(values: Sequence[int]) -> str:
    return " ".join(
        f"M{index}={'n/a' if value < 0 else value}"
        for index, value in enumerate(values, start=1)
    )


class MissionState(str, Enum):
    PREFLIGHT = "PREFLIGHT"
    CALIBRATE = "CALIBRATE"
    WAIT_TAKEOVER = "WAIT_TAKEOVER"
    ARM = "ARM"
    BENCH_ARM_TEST = "BENCH_ARM_TEST"
    TAKEOFF = "TAKEOFF"
    ALTITUDE_HOVER = "ALTITUDE_HOVER"
    GPS_ACQUIRE = "GPS_ACQUIRE"
    GPS_HOLD = "GPS_HOLD"
    LAND = "LAND"
    DISARM = "DISARM"
    COMPLETE = "COMPLETE"
    MANUAL = "MANUAL"
    ABORT = "ABORT"


class FlightLog:
    FIELDS = (
        "gps_sample_valid", "gps_raw_north_m", "gps_raw_east_m", "position_source",
        "relative_north_m", "relative_east_m",
        "flow_quality", "flow_status", "flow_x_dps", "flow_y_dps",
        "flow_body_x_dps", "flow_body_y_dps", "flow_range_m",
        "flow_valid", "flow_reason", "flow_north_mps", "flow_east_mps",
        "flow_weight", "velocity_source", "gps_velocity_north_mps", "gps_velocity_east_mps",
        "flow_watchdog", "local_pose_north_m", "local_pose_east_m",
        "local_pose_velocity_north_mps", "local_pose_velocity_east_mps",
        "time_s",
        "mission_mode",
        "state",
        "event",
        "active_modes",
        "position_pid_enabled",
        "height_agl_m",
        "height_raw_m",
        "rangefinder_distance_m",
        "height_setpoint_m",
        "vertical_speed_mps",
        "inav_vertical_speed_mps",
        "gps_calib_lat",
        "gps_calib_lon",
        "gps_measurement_lat",
        "gps_measurement_lon",
        "gps_lat",
        "gps_lon",
        "north_m",
        "east_m",
        "position_error_m",
        "horizontal_control_mode",
        "velocity_north_setpoint_mps",
        "velocity_east_setpoint_mps",
        "velocity_north_mps",
        "velocity_east_mps",
        "velocity_north_error_mps",
        "velocity_east_error_mps",
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
        "roll_deg",
        "pitch_deg",
        "yaw_deg",
        "roll_pwm",
        "pitch_pwm",
        "throttle_pwm",
        "alt_p",
        "alt_i",
        "alt_d",
        "gps_fix",
        "satellites",
        "hdop",
        "motor_1_output",
        "motor_2_output",
        "motor_3_output",
        "motor_4_output",
        "motor_1_rpm",
        "motor_2_rpm",
        "motor_3_rpm",
        "motor_4_rpm",
    )

    def __init__(self, directory: Path, start_time: float) -> None:
        directory.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = directory / f"mission_{stamp}.csv"
        self._file: TextIO = self.path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=self.FIELDS)
        self._writer.writeheader()
        self.start_time = start_time

    def write(self, row: dict[str, object]) -> None:
        self._writer.writerow({key: row.get(key, "") for key in self.FIELDS})
        self._file.flush()

    def close(self) -> None:
        self._file.close()


class CalibrationLog:
    FIELDS = (
        "record_type",
        "sample_index",
        "time_s",
        "roll_deg",
        "pitch_deg",
        "yaw_deg",
        "height_raw_m",
        "rangefinder_distance_m",
        "gps_lat",
        "gps_lon",
        "gps_fix",
        "satellites",
        "hdop",
        "height_std_m",
        "position_rms_m",
        "roll_method",
        "pitch_method",
        "yaw_method",
        "height_method",
        "gps_method",
    )

    def __init__(self, directory: Path, start_time: float) -> None:
        directory.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = directory / f"calibration_{stamp}.csv"
        self._file: TextIO = self.path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=self.FIELDS)
        self._writer.writeheader()
        self.start_time = start_time

    def write(self, row: dict[str, object]) -> None:
        self._writer.writerow({key: row.get(key, "") for key in self.FIELDS})
        self._file.flush()

    def close(self) -> None:
        self._file.close()


class MissionController:
    """Preflight-to-disarm mission with altitude and horizontal control."""

    def __init__(
        self,
        vehicle: Vehicle,
        config: AppConfig,
        *,
        clock: Clock | None = None,
        fly: bool = False,
        simulation: bool = False,
        altitude_only: bool = False,
        bench_arm_test: bool = False,
        confirmation: Callable[[str], str] = input,
        output: Callable[[str], None] = print,
        log_root: str | Path = ".",
    ) -> None:
        self.vehicle = vehicle
        self.config = config
        self.clock = clock or RealClock()
        self.fly = fly
        self.simulation = simulation
        self.altitude_only = altitude_only
        self.gps_enabled = not altitude_only
        self.relative_mode = self.gps_enabled and config.optical_flow.mode == "relative"
        self.bench_arm_test_enabled = bench_arm_test
        if self.bench_arm_test_enabled and not self.altitude_only:
            raise ValueError("bench_arm_test requires altitude_only channel ownership")
        self.confirmation = confirmation
        self.output = output
        self.log_root = Path(log_root)
        self.state = MissionState.PREFLIGHT
        self.ground: GroundReference | None = None
        self.ranges = []
        self.rx_map: list[int] = []
        self.channel_count = 12
        self.takeover_index = config.safety.takeover_channel - 1
        self.prearm_modes: set[str] = {"ANGLE"}
        self.armed_modes: set[str] = {"ANGLE", "ARM"}
        self.current_throttle = float(config.control.rc_min)
        self.altitude_pid = PID(config.control.altitude_pid)
        velocity_config = PIDConfig(config.control.velocity_tilt_kp,
                                    config.control.velocity_tilt_ki, 0.0,
                                    config.control.max_tilt_deg, 2.5, 0.0)
        self.north_pid = PID(velocity_config)
        self.east_pid = PID(velocity_config)
        self._log: FlightLog | None = None
        self._calibration_log_path: Path | None = None
        self._last_console = -math.inf
        self._last_mode_check = -math.inf
        self._mission_started = 0.0
        self._armed = False
        self._last_active_modes: set[str] = set()
        self._last_height_velocity_sample: tuple[float, float] | None = None
        self._filtered_height_velocity_mps = 0.0
        self._motor_outputs: tuple[int, int, int, int] = (-1, -1, -1, -1)
        self._next_motor_output_poll = -math.inf
        self._motor_output_warning_emitted = False
        self._motor_rpm: tuple[int, int, int, int] = (-1, -1, -1, -1)
        self._next_motor_rpm_poll = -math.inf
        self._motor_rpm_available = True
        self._flow_watchdog = FlowWatchdog(config.optical_flow.flow_loss_timeout_s)
        self._flow_sensor_status = HW_SENSOR_OK
        self._next_flow_status_poll = -math.inf
        self._gps_reference_available = False
        self._local_pose_xy_origin: tuple[float, float] | None = None
        self._local_pose_z_origin: float | None = None

    def _set_state(self, state: MissionState) -> None:
        self.state = state
        self.output(f"\n=== {state.value} ===")

    def _gps_ok(self, gps) -> bool:
        s = self.config.sensors
        return (
            gps.fix_type == GPS_FIX_3D
            and gps.satellites >= s.min_satellites
            and 0 < gps.hdop <= s.max_hdop
            and -90 <= gps.lat <= 90
            and -180 <= gps.lon <= 180
            and (abs(gps.lat) > 1e-8 or abs(gps.lon) > 1e-8)
        )

    def _height_raw(self) -> float:
        sensors = self.config.sensors
        if sensors.height_source == "rangefinder":
            height = self.vehicle.rangefinder_m()
            if not sensors.min_range_m <= height <= sensors.max_range_m:
                raise SafetyStop(
                    f"Rangefinder value {height:.3f} m outside "
                    f"{sensors.min_range_m:.2f}..{sensors.max_range_m:.2f} m"
                )
            return height
        height = self.vehicle.altitude().barometer_alt_m
        if not math.isfinite(height):
            raise SafetyStop("Barometer altitude is not finite")
        return height

    def _height_vertical_speed(
        self,
        height_raw_m: float,
        sample_time: float,
        inav_vertical_speed_mps: float,
    ) -> float:
        """Use a velocity estimate consistent with the selected height source."""
        if self.config.sensors.height_source != "rangefinder":
            return inav_vertical_speed_mps
        previous = self._last_height_velocity_sample
        self._last_height_velocity_sample = (sample_time, height_raw_m)
        if previous is None:
            self._filtered_height_velocity_mps = 0.0
            return 0.0
        previous_time, previous_height = previous
        dt = sample_time - previous_time
        if dt <= 0:
            return self._filtered_height_velocity_mps
        raw_velocity = (height_raw_m - previous_height) / dt
        cutoff_hz = self.config.sensors.height_velocity_filter_hz
        tau = 1.0 / (2.0 * math.pi * cutoff_hz)
        alpha = dt / (tau + dt)
        self._filtered_height_velocity_mps += alpha * (
            raw_velocity - self._filtered_height_velocity_mps
        )
        return self._filtered_height_velocity_mps

    def _poll_motor_rpm(self, *, force: bool = False) -> tuple[int, int, int, int]:
        now = self.clock.monotonic()
        if not force and now < self._next_motor_rpm_poll:
            return self._motor_rpm
        if not self._motor_rpm_available and not self.config.logging.require_motor_rpm:
            return self._motor_rpm
        try:
            values = self.vehicle.motor_rpm()
            if len(values) < 4:
                raise ValueError(f"expected 4 motors, received {len(values)}")
            self._motor_rpm = tuple(int(value) for value in values[:4])
            if any(value < 0 for value in self._motor_rpm):
                raise ValueError("motor RPM cannot be negative")
            self._motor_rpm_available = True
        except (MSPError, OSError, ValueError) as exc:
            if self.config.logging.require_motor_rpm:
                raise SafetyStop(f"Four-motor ESC RPM telemetry unavailable: {exc}") from exc
            self._motor_rpm_available = False
            self._motor_rpm = (-1, -1, -1, -1)
            self.output(f"[WARN] ESC RPM telemetry unavailable; RPM columns will be blank: {exc}")
        self._next_motor_rpm_poll = now + 1.0 / self.config.logging.motor_rpm_hz
        return self._motor_rpm

    def _poll_motor_outputs(self, *, force: bool = False) -> tuple[int, int, int, int]:
        """Poll FC-to-ESC commands as optional diagnostics, never as a flight gate."""
        now = self.clock.monotonic()
        if not force and now < self._next_motor_output_poll:
            return self._motor_outputs
        try:
            values = self.vehicle.motor_outputs()
            if len(values) < 4:
                raise ValueError(f"expected 4 motors, received {len(values)}")
            self._motor_outputs = tuple(int(value) for value in values[:4])
            self._motor_output_warning_emitted = False
        except (MSPError, OSError, ValueError) as exc:
            self._motor_outputs = (-1, -1, -1, -1)
            if not self._motor_output_warning_emitted:
                self.output(f"[WARN] MSP_MOTOR output unavailable: {exc}")
                self._motor_output_warning_emitted = True
        self._next_motor_output_poll = now + 1.0 / self.config.logging.motor_rpm_hz
        return self._motor_outputs

    def preflight(self) -> None:
        self._set_state(MissionState.PREFLIGHT)
        startup_deadline = (
            self.clock.monotonic() + self.config.serial.startup_timeout_s
        )
        startup_error: Exception | None = None
        attempt = 0
        while self.clock.monotonic() < startup_deadline:
            attempt += 1
            try:
                api = self.vehicle.api_version()
                break
            except (MSPError, OSError) as exc:
                startup_error = exc
                if attempt == 1:
                    self.output(
                        "Waiting for bidirectional MSP telemetry response "
                        f"for up to {self.config.serial.startup_timeout_s:.1f} s..."
                    )
                self.clock.sleep(self.config.serial.startup_retry_delay_s)
        else:
            raise SafetyStop(
                "No bidirectional MSP telemetry response during startup; "
                "RC write-only may work, but closed-loop altitude control cannot run. "
                f"Last error: {startup_error}"
            ) from startup_error
        variant = self.vehicle.fc_variant()
        version = self.vehicle.fc_version()
        self.output(
            f"FC={variant} {version[0]}.{version[1]}.{version[2]} | "
            f"MSP protocol={api.protocol}, API={api.major}.{api.minor}"
        )
        if variant != "INAV" or api.major != 2:
            raise SafetyStop("This controller requires INAV with MSP API major 2")

        status = self.vehicle.sensor_status()
        if self.config.optical_flow.mode != "off" and self.gps_enabled:
            if status.optical_flow != HW_SENSOR_OK:
                raise SafetyStop("Optical flow sensor is not healthy")
            self.vehicle.optical_flow()  # Verify protocol before ARM.
            if self.config.optical_flow.mode == "relative":
                pose = self.vehicle.local_pose()
                if not all(math.isfinite(value) for value in asdict(pose).values()):
                    raise SafetyStop("INAV local pose is not finite")
        required = {
            "gyro": status.gyro,
            "accelerometer": status.accelerometer,
            self.config.sensors.height_source: (
                status.rangefinder
                if self.config.sensors.height_source == "rangefinder"
                else status.barometer
            ),
        }
        gps_required = self.gps_enabled and self.config.optical_flow.mode != "relative"
        if gps_required:
            required["compass"] = status.compass
            required["gps"] = status.gps
        bad = [name for name, value in required.items() if value != HW_SENSOR_OK]
        if bad:
            raise SafetyStop("Required INAV sensor(s) are not HW_SENSOR_OK: " + ", ".join(bad))
        if not status.overall_healthy and gps_required:
            raise SafetyStop("INAV reports unhealthy configured hardware")

        height = self._height_raw()
        if self.relative_mode:
            pose = self.vehicle.local_pose()
            attitude = AttitudeData(pose.roll_deg, pose.pitch_deg, pose.yaw_deg)
        else:
            attitude = self.vehicle.attitude()
        if max(abs(attitude.roll_deg), abs(attitude.pitch_deg)) > self.config.sensors.max_ground_tilt_deg:
            raise SafetyStop("Drone is not sufficiently level for ground calibration")
        if self.gps_enabled:
            gps = self.vehicle.gps()
            if not self._gps_ok(gps) and gps_required:
                raise SafetyStop(
                    f"GPS gate failed: fix={gps.fix_type}, sats={gps.satellites}, "
                    f"HDOP={gps.hdop:.2f}"
                )
            gps_text = (f"GPS={gps.lat:.7f},{gps.lon:.7f} sats={gps.satellites} "
                        f"HDOP={gps.hdop:.2f}" if self._gps_ok(gps)
                        else "GPS unavailable; relative flow will run without GPS correction")
            self.output(f"Sensors OK | {gps_text} | height raw={height:.3f} m")
        else:
            self.output(
                "Sensors OK | GPS/compass not required in altitude-only mode | "
                f"height raw={height:.3f} m"
            )
        outputs = self._poll_motor_outputs(force=True)
        rpm = self._poll_motor_rpm(force=True)
        self.output(f"FC motor output [{format_motor_outputs(outputs)}]")
        self.output(f"ESC RPM telemetry [{format_motor_rpm(rpm)}]")

        if "ARM" in self.vehicle.active_modes():
            raise SafetyStop("Preflight requires INAV to be DISARMED")

        self.ranges = self.vehicle.mode_ranges()
        validate_mode_configuration(
            self.ranges, self.takeover_index, self.config.safety.takeover_on_pwm
        )
        self.rx_map = self.vehicle.rx_map()
        self.channel_count = max(
            12,
            len(self.rx_map),
            max(item.channel_index for item in self.ranges) + 1,
        )
        if "PREARM" in ranges_by_name(self.ranges):
            self.prearm_modes.add("PREARM")
            self.armed_modes.add("PREARM")

        rc = self.vehicle.rc_channels()
        if len(rc) <= self.takeover_index:
            raise SafetyStop(f"MSP_RC does not contain CH{self.takeover_index + 1}")
        if rc[self.takeover_index] > self.config.safety.takeover_off_pwm:
            raise SafetyStop("Start with the physical MSP takeover switch OFF")

    def calibrate_ground(self) -> GroundReference:
        self._set_state(MissionState.CALIBRATE)
        s = self.config.sensors
        rolls: list[float] = []
        pitches: list[float] = []
        yaws: list[float] = []
        lats: list[float] = []
        lons: list[float] = []
        heights: list[float] = []
        rangefinder_distances: list[float] = []
        log_dir = self.log_root / self.config.logging.directory
        calibration_start = self.clock.monotonic()
        calibration_log = CalibrationLog(log_dir, calibration_start)
        self._calibration_log_path = calibration_log.path
        self.output(f"Calibration samples: {calibration_log.path}")
        try:
            for sample_index in range(1, s.calibration_samples + 1):
                if self.relative_mode:
                    pose = self.vehicle.local_pose()
                    attitude = AttitudeData(pose.roll_deg, pose.pitch_deg, pose.yaw_deg)
                    height = pose.up_m
                    rangefinder_distance = self._height_raw()
                else:
                    attitude = self.vehicle.attitude()
                    height = self._height_raw()
                    rangefinder_distance = (
                        height if s.height_source == "rangefinder" else math.nan
                    )
                gps = self.vehicle.gps() if self.gps_enabled else None
                calibration_log.write(
                    {
                        "record_type": "SAMPLE",
                        "sample_index": sample_index,
                        "time_s": self.clock.monotonic() - calibration_start,
                        "roll_deg": attitude.roll_deg,
                        "pitch_deg": attitude.pitch_deg,
                        "yaw_deg": attitude.yaw_deg,
                        "height_raw_m": height,
                        "rangefinder_distance_m": rangefinder_distance,
                        "gps_lat": gps.lat if gps is not None else "",
                        "gps_lon": gps.lon if gps is not None else "",
                        "gps_fix": gps.fix_type if gps is not None else "",
                        "satellites": gps.satellites if gps is not None else "",
                        "hdop": gps.hdop if gps is not None else "",
                    }
                )
                if (gps is not None and not self._gps_ok(gps)
                        and self.config.optical_flow.mode != "relative"):
                    raise SafetyStop("GPS quality changed during ground calibration")
                rolls.append(attitude.roll_deg)
                pitches.append(attitude.pitch_deg)
                yaws.append(attitude.yaw_deg)
                if gps is not None and self._gps_ok(gps):
                    lats.append(gps.lat)
                    lons.append(gps.lon)
                heights.append(height)
                if math.isfinite(rangefinder_distance):
                    rangefinder_distances.append(rangefinder_distance)
                self.clock.sleep(1.0 / s.calibration_hz)

            reference = GroundReference(
                roll_deg=statistics.mean(rolls),
                pitch_deg=statistics.mean(pitches),
                yaw_deg=circular_mean_deg(yaws),
                lat=statistics.median(lats) if lats else 0.0,
                lon=statistics.median(lons) if lons else 0.0,
                height_raw_m=statistics.median(heights),
                rangefinder_distance_m=(statistics.median(rangefinder_distances)
                                        if rangefinder_distances else 0.0),
            )
            height_std = statistics.pstdev(heights)
            position_samples = [
                local_ne_m(reference.lat, reference.lon, lat, lon)
                for lat, lon in zip(lats, lons)
            ]
            position_rms = (
                math.sqrt(
                    statistics.mean(
                        north * north + east * east
                        for north, east in position_samples
                    )
                )
                if position_samples
                else 0.0
            )
            calibration_log.write(
                {
                    "record_type": "REFERENCE",
                    "sample_index": s.calibration_samples,
                    "time_s": self.clock.monotonic() - calibration_start,
                    "roll_deg": reference.roll_deg,
                    "pitch_deg": reference.pitch_deg,
                    "yaw_deg": reference.yaw_deg,
                    "height_raw_m": reference.height_raw_m,
                    "rangefinder_distance_m": reference.rangefinder_distance_m,
                    "gps_lat": reference.lat if lats else "",
                    "gps_lon": reference.lon if lons else "",
                    "height_std_m": height_std,
                    "position_rms_m": position_rms if self.gps_enabled else "",
                    "roll_method": "mean",
                    "pitch_method": "mean",
                    "yaw_method": "circular_mean",
                    "height_method": "median",
                    "gps_method": "median",
                }
            )
        finally:
            calibration_log.close()
        if height_std > s.max_height_std_m:
            raise SafetyStop(f"Height sensor unstable on ground: std={height_std:.3f} m")
        if (self.gps_enabled and self.config.optical_flow.mode != "relative"
                and position_rms > s.max_position_std_m):
            raise SafetyStop(f"GPS unstable on ground: RMS={position_rms:.2f} m")
        if max(abs(reference.roll_deg), abs(reference.pitch_deg)) > s.max_ground_tilt_deg:
            raise SafetyStop("Mean ground attitude exceeds tilt limit")
        if (s.height_source == "rangefinder"
                and reference.rangefinder_distance_m + self.config.control.target_altitude_m >= s.max_range_m):
            raise SafetyStop("Target altitude exceeds the calibrated rangefinder envelope")
        self.ground = reference
        self._gps_reference_available = len(lats) == s.calibration_samples
        self._last_height_velocity_sample = None
        self._filtered_height_velocity_mps = 0.0
        self.output(
            f"Ground attitude roll={reference.roll_deg:.2f}°, pitch={reference.pitch_deg:.2f}°, "
            f"yaw={reference.yaw_deg:.2f}°"
        )
        if self.gps_enabled and self._gps_reference_available:
            self.output(
                f"Ground GPS={reference.lat:.7f},{reference.lon:.7f} | "
                f"INAV local Z={reference.height_raw_m:.3f} m | "
                f"GPS RMS={position_rms:.2f} m"
            )
        elif self.gps_enabled:
            self.output(
                f"INAV local Z={reference.height_raw_m:.3f} m | GPS monitoring unavailable"
            )
        else:
            self.output(
                f"Ground height raw={reference.height_raw_m:.3f} m | GPS disabled"
            )
        return reference

    def _frame(
        self,
        *,
        armed: bool,
        throttle: int,
        roll: int = 1500,
        pitch: int = 1500,
        yaw: int = 1500,
    ) -> list[int]:
        modes = self.armed_modes if armed else self.prearm_modes
        return choose_channels(
            self.ranges,
            modes,
            self.channel_count,
            self.takeover_index,
            roll=roll,
            pitch=pitch,
            throttle=throttle,
            yaw=yaw,
        )

    def wait_for_takeover(self) -> None:
        self._set_state(MissionState.WAIT_TAKEOVER)
        timeout = self.config.safety.takeover_wait_timeout_s
        deadline = self.clock.monotonic() + timeout
        self.output(
            f"Turn physical CH{self.config.safety.takeover_channel} takeover ON "
            f"(>= {self.config.safety.takeover_on_pwm})."
        )
        while self.clock.monotonic() < deadline:
            rc = self.vehicle.rc_channels()
            modes = self.vehicle.active_modes()
            if (
                len(rc) > self.takeover_index
                and rc[self.takeover_index] >= self.config.safety.takeover_on_pwm
                and "MSP RC OVERRIDE" in modes
            ):
                self.output("Physical takeover permission accepted.")
                return
            self.clock.sleep(0.1)
        raise SafetyStop("Physical takeover was not enabled before timeout")

    def _send(self, channels: Sequence[int]) -> None:
        self.vehicle.set_raw_rc(channels, self.rx_map)

    def _stream(self, duration_s: float, channels: Sequence[int]) -> None:
        period = 1.0 / self.config.control.loop_hz
        deadline = self.clock.monotonic() + duration_s
        while self.clock.monotonic() < deadline:
            started = self.clock.monotonic()
            self._send(channels)
            self.clock.sleep(max(0.0, period - (self.clock.monotonic() - started)))

    def verify_takeover(self) -> None:
        if self.altitude_only:
            self._verify_altitude_only_override()
            return
        safe = self._frame(armed=False, throttle=self.config.control.rc_min)
        self._stream(self.config.safety.override_settle_s, safe)
        probe_values = (
            self.config.control.rc_mid + 125,
            self.config.control.rc_mid - 125,
            self.config.control.rc_mid + 150,
            min(self.config.control.rc_min + 100, 1200),
        )
        probe = self._frame(
            armed=False,
            roll=probe_values[0],
            pitch=probe_values[1],
            yaw=probe_values[2],
            throttle=probe_values[3],
        )
        self._stream(0.8, probe)
        self.clock.sleep(0.1)
        rc = self.vehicle.rc_channels()
        self._stream(0.5, safe)
        if len(rc) < 4:
            raise SafetyStop("MSP_RC did not return the four primary RC channels")
        modes = self.vehicle.active_modes()
        self._last_active_modes = set(modes)
        if any("FAILSAFE" in mode.upper() for mode in modes):
            raise SafetyStop("INAV FAILSAFE mode became active")
        if "MSP RC OVERRIDE" not in modes:
            raise SafetyStop(
                "Takeover disappeared while MSP frame held its own takeover channel LOW; "
                "exclude that physical channel from msp_override_channels"
            )
        if "ANGLE" not in modes or "ARM" in modes:
            raise SafetyStop(f"Disarmed takeover verification failed; active modes={sorted(modes)}")
        if any(abs(actual - expected) > 35 for actual, expected in zip(rc[:4], probe_values)):
            raise SafetyStop(
                "Full-GPS override verification failed: Pi does not own all AERT "
                f"channels (expected {probe_values}, received {tuple(rc[:4])}). "
                "Configure INAV msp_override_channels = 63"
            )
        self.output(
            "Override verified: Pi owns roll/pitch/yaw/throttle + ARM/ANGLE for full-GPS."
        )

    def _verify_altitude_only_override(self) -> None:
        """Prove the throttle-only/manual-attitude override layout before arming."""
        safe = self._frame(armed=False, throttle=self.config.control.rc_min)
        self.output(
            "Settling MSP override with ARM OFF and throttle LOW for "
            f"{self.config.safety.override_settle_s:.1f} s."
        )
        self._stream(self.config.safety.override_settle_s, safe)
        probe_roll = self.config.control.rc_mid + 125
        probe_pitch = self.config.control.rc_mid - 125
        probe_yaw = self.config.control.rc_mid + 150
        probe_throttle = min(self.config.control.rc_min + 100, 1200)
        probe = self._frame(
            armed=False,
            throttle=probe_throttle,
            roll=probe_roll,
            pitch=probe_pitch,
            yaw=probe_yaw,
        )
        self._stream(0.8, probe)
        self.clock.sleep(0.1)
        rc = self.vehicle.rc_channels()
        self._stream(0.5, safe)
        if len(rc) < 4:
            raise SafetyStop("MSP_RC did not return the four primary RC channels")
        modes = self.vehicle.active_modes()
        if "MSP RC OVERRIDE" not in modes or "ANGLE" not in modes or "ARM" in modes:
            raise SafetyStop(
                f"Altitude-only takeover verification failed; active modes={sorted(modes)}"
            )
        if abs(rc[3] - probe_throttle) > 35:
            raise SafetyStop(
                f"Throttle is not overridden by MSP (expected {probe_throttle}, "
                f"received AERT={rc[:4]}). For AETR1234, configure "
                "msp_override_channels = 24 and allow the override failsafe to settle"
            )
        tolerance = self.config.safety.manual_stick_center_tolerance_pwm
        manual = rc[:3]
        if any(abs(value - self.config.control.rc_mid) > tolerance for value in manual):
            if (
                abs(rc[0] - probe_roll) <= 35
                or abs(rc[1] - probe_pitch) <= 35
                or abs(rc[2] - probe_yaw) <= 35
            ):
                raise SafetyStop(
                    "Roll/pitch/yaw are still overridden by MSP. Altitude-only mode "
                    "requires msp_override_channels = 24 for AETR1234"
                )
            raise SafetyStop(
                "Center the physical roll/pitch/yaw sticks during altitude-only "
                f"verification (received {manual})"
            )
        self.output(
            "Override verified: Pi owns throttle + ARM; physical RC owns "
            "roll/pitch/yaw and ANGLE is active."
        )

    def arm(self) -> None:
        self._set_state(MissionState.ARM)
        low = self._frame(armed=True, throttle=self.config.control.rc_min)
        deadline = self.clock.monotonic() + self.config.safety.arm_timeout_s
        period = 1.0 / self.config.control.loop_hz
        while self.clock.monotonic() < deadline:
            self._send(low)
            modes = self.vehicle.active_modes()
            if "MSP RC OVERRIDE" not in modes:
                raise ManualTakeover("Physical takeover switch is OFF during ARM")
            if "ARM" in modes:
                self._armed = True
                self._local_pose_xy_origin = None
                self._local_pose_z_origin = None
                self.current_throttle = float(self.config.control.rc_min)
                self.output("INAV confirmed ARMED with throttle LOW.")
                return
            self.clock.sleep(period)
        raise SafetyStop("INAV did not ARM; inspect INAV arming-disable flags")

    def bench_arm_test(self) -> None:
        """Hold minimum throttle briefly so a props-off setup can verify arming."""
        self._set_state(MissionState.BENCH_ARM_TEST)
        duration = self.config.safety.bench_arm_hold_s
        deadline = self.clock.monotonic() + duration
        period = 1.0 / self.config.control.loop_hz
        frame = self._frame(armed=True, throttle=self.config.control.rc_min)
        self.output(
            f"Holding ARMED at minimum throttle for {duration:.1f} s; props must be removed."
        )
        while self.clock.monotonic() < deadline:
            started = self.clock.monotonic()
            self._send(frame)
            self._check_control_permission()
            telemetry = self._telemetry()
            assert self.ground is not None
            north = east = 0.0
            output_text = format_motor_outputs(telemetry.motor_outputs)
            rpm_text = format_motor_rpm(telemetry.motor_rpm)
            if self._log is not None:
                self._log.write(
                    {
                        "time_s": telemetry.timestamp - self._mission_started,
                        "mission_mode": "BENCH_ARM_TEST",
                        "state": self.state.value,
                        "event": "",
                        "active_modes": "|".join(sorted(self._last_active_modes)),
                        "position_pid_enabled": 0,
                        "height_agl_m": telemetry.height_agl_m,
                        "height_raw_m": telemetry.height_raw_m,
                        "height_setpoint_m": 0.0,
                        "vertical_speed_mps": telemetry.vertical_speed_mps,
                        "inav_vertical_speed_mps": telemetry.inav_vertical_speed_mps,
                        "gps_lat": "",
                        "gps_lon": "",
                        "north_m": "",
                        "east_m": "",
                        "position_error_m": "",
                        "roll_deg": telemetry.attitude.roll_deg,
                        "pitch_deg": telemetry.attitude.pitch_deg,
                        "yaw_deg": telemetry.attitude.yaw_deg,
                        "roll_pwm": self.config.control.rc_mid,
                        "pitch_pwm": self.config.control.rc_mid,
                        "throttle_pwm": self.config.control.rc_min,
                        "motor_1_output": telemetry.motor_outputs[0],
                        "motor_2_output": telemetry.motor_outputs[1],
                        "motor_3_output": telemetry.motor_outputs[2],
                        "motor_4_output": telemetry.motor_outputs[3],
                        "motor_1_rpm": telemetry.motor_rpm[0],
                        "motor_2_rpm": telemetry.motor_rpm[1],
                        "motor_3_rpm": telemetry.motor_rpm[2],
                        "motor_4_rpm": telemetry.motor_rpm[3],
                        "gps_fix": "",
                        "satellites": "",
                        "hdop": "",
                    }
                )
            if telemetry.timestamp - self._last_console >= self.config.logging.console_period_s:
                self.output(
                    f"BENCH height={telemetry.height_agl_m:.2f} m "
                    "GPS=disabled "
                    f"OUT[{output_text}] RPM[{rpm_text}] "
                    f"throttle={self.config.control.rc_min}"
                )
                self._last_console = telemetry.timestamp
            self.clock.sleep(max(0.0, period - (self.clock.monotonic() - started)))
        self.output("Bench hold complete; sending DISARM now.")

    def _telemetry(self) -> Telemetry:
        if self.ground is None:
            raise RuntimeError("Ground reference has not been calibrated")
        local_pose = self.vehicle.local_pose() if self.relative_mode else None
        if local_pose is not None:
            if not all(math.isfinite(value) for value in asdict(local_pose).values()):
                raise SafetyStop("INAV local pose became non-finite")
            attitude = AttitudeData(
                local_pose.roll_deg, local_pose.pitch_deg, local_pose.yaw_deg
            )
        else:
            attitude = self.vehicle.attitude()
        gps = (
            self.vehicle.gps()
            if self.gps_enabled
            else GPSData(0, 0, 0.0, 0.0, 0.0, 0.0)
        )
        if local_pose is not None:
            rangefinder_distance = self._height_raw()
            height_raw = local_pose.up_m
            vertical_speed = local_pose.velocity_up_mps
            inav_vertical_speed = local_pose.velocity_up_mps
            if self._local_pose_z_origin is None:
                self._local_pose_z_origin = local_pose.up_m
        else:
            altitude = self.vehicle.altitude()
            height_raw = self._height_raw()
            rangefinder_distance = (
                height_raw if self.config.sensors.height_source == "rangefinder"
                else math.nan
            )
            height_sample_time = self.clock.monotonic()
            vertical_speed = self._height_vertical_speed(
                height_raw,
                height_sample_time,
                altitude.vertical_speed_mps,
            )
            inav_vertical_speed = altitude.vertical_speed_mps
        motor_outputs = self._poll_motor_outputs()
        motor_rpm = self._poll_motor_rpm()
        if (self.gps_enabled and not self._gps_ok(gps)
                and self.config.optical_flow.mode != "relative"):
            raise SafetyStop(
                f"GPS lost: fix={gps.fix_type}, sats={gps.satellites}, HDOP={gps.hdop:.2f}"
            )
        height_origin = (
            self._local_pose_z_origin
            if local_pose is not None and self._local_pose_z_origin is not None
            else self.ground.height_raw_m
        )
        height_agl = height_raw - height_origin
        if height_agl < -self.config.control.ground_detect_margin_m:
            raise SafetyStop(f"Height below calibrated ground: {height_agl:.2f} m")
        if max(abs(attitude.roll_deg), abs(attitude.pitch_deg)) > self.config.safety.max_flight_tilt_deg:
            raise SafetyStop(
                f"Unsafe tilt: roll={attitude.roll_deg:.1f}°, pitch={attitude.pitch_deg:.1f}°"
            )
        if abs(vertical_speed) > self.config.safety.max_vertical_speed_mps:
            raise SafetyStop(
                f"Unsafe {self.config.sensors.height_source} vertical speed: "
                f"{vertical_speed:.2f} m/s (INAV={inav_vertical_speed:.2f} m/s)"
            )
        return Telemetry(
            self.clock.monotonic(),
            attitude,
            gps,
            height_raw,
            max(0.0, height_agl),
            vertical_speed,
            inav_vertical_speed,
            motor_outputs,
            motor_rpm,
            rangefinder_distance,
            local_pose,
        )

    def _check_control_permission(self) -> None:
        now = self.clock.monotonic()
        if (
            self._mission_started
            and now - self._mission_started > self.config.safety.maximum_mission_time_s
        ):
            raise SafetyStop("Maximum mission time exceeded")
        if now - self._last_mode_check < 0.5:
            return
        modes = self.vehicle.active_modes()
        self._last_active_modes = set(modes)
        if any("FAILSAFE" in mode.upper() for mode in modes):
            raise SafetyStop("INAV FAILSAFE mode became active")
        if "MSP RC OVERRIDE" not in modes:
            raise ManualTakeover("Physical takeover switch turned OFF")
        missing = self.armed_modes - modes
        if missing:
            raise SafetyStop("Required flight mode(s) missing: " + ", ".join(sorted(missing)))
        conflicting = {"NAV ALTHOLD", "NAV POSHOLD", "NAV WP", "GCS NAV"} & modes
        if conflicting:
            raise SafetyStop("Conflicting INAV navigation mode(s) active: " + ", ".join(sorted(conflicting)))
        self._last_mode_check = now

    def _control_step(
        self,
        telemetry: Telemetry,
        height_setpoint: float,
        dt: float,
        *,
        position_enabled: bool,
        altitude_integrate: bool = True,
        position_tilt_limit_deg: float | None = None,
    ) -> dict[str, object]:
        assert self.ground is not None
        if dt > self.config.safety.max_control_gap_s:
            raise SafetyStop(
                f"Control-loop gap {dt:.3f} s exceeds "
                f"{self.config.safety.max_control_gap_s:.3f} s"
            )
        if position_enabled and not self.gps_enabled:
            raise SafetyStop("Horizontal position control is unavailable in altitude-only mode")
        f = self.config.optical_flow
        gps_sample_valid = self.gps_enabled and self._gps_reference_available and self._gps_ok(telemetry.gps)
        if gps_sample_valid:
            north, east = local_ne_m(
                self.ground.lat, self.ground.lon, telemetry.gps.lat, telemetry.gps.lon
            )
        else:
            north = east = 0.0
        position_error = math.hypot(north, east)
        gps_raw_north, gps_raw_east = north, east
        if f.mode != "relative" and position_error > self.config.safety.max_position_error_m:
            raise SafetyStop(
                f"Position error {position_error:.1f} m exceeds "
                f"{self.config.safety.max_position_error_m:.1f} m"
            )

        gps_north_error: float | str = ""
        gps_north_p: float | str = ""
        gps_north_i: float | str = ""
        gps_north_d: float | str = ""
        gps_north_output: float | str = ""
        gps_east_error: float | str = ""
        gps_east_p: float | str = ""
        gps_east_i: float | str = ""
        gps_east_d: float | str = ""
        gps_east_output: float | str = ""
        tilt_north_limited: float | str = ""
        tilt_east_limited: float | str = ""
        forward_tilt: float | str = ""
        right_tilt: float | str = ""

        velocity_log: dict[str, object] = {}
        flow_log: dict[str, object] = {}
        flow_n = flow_e = 0.0
        # Below the flow operating range on final descent, level the aircraft
        # instead of integrating an unobservable horizontal position.
        if (f.mode == "relative" and self.state == MissionState.LAND
                and telemetry.rangefinder_distance_m < f.min_range_m):
            position_enabled = False
        if f.mode != "off" and self.gps_enabled:
            request_started = self.clock.monotonic()
            sample = self.vehicle.optical_flow()
            now = self.clock.monotonic()
            if now >= self._next_flow_status_poll:
                self._flow_sensor_status = self.vehicle.sensor_status().optical_flow
                self._next_flow_status_poll = now + 0.5
            status = self._flow_sensor_status
            if self.clock.monotonic() - request_started > self.config.safety.max_control_gap_s or self.clock.monotonic() - telemetry.timestamp > self.config.safety.max_control_gap_s:
                raise SafetyStop("Optical flow polling exceeded control freshness limit")
            flow_n, flow_e, flow_log = flow_velocity(
                sample, status, telemetry.rangefinder_distance_m,
                telemetry.attitude, f, check_speed=f.mode != "relative")
            watchdog = self._flow_watchdog.update(bool(flow_log["flow_valid"]), now)
            flow_log["flow_watchdog"] = watchdog
            if f.mode == "relative":
                pose = telemetry.local_pose
                if pose is None:
                    raise SafetyStop("INAV local pose is missing")
                if math.hypot(pose.velocity_north_mps, pose.velocity_east_mps) > f.local_pose_max_speed_mps:
                    raise SafetyStop("INAV local-pose horizontal speed exceeds safety limit")
                flow_log.update(
                    local_pose_north_m=pose.north_m,
                    local_pose_east_m=pose.east_m,
                    local_pose_velocity_north_mps=pose.velocity_north_mps,
                    local_pose_velocity_east_mps=pose.velocity_east_mps,
                )
                flow_n, flow_e = pose.velocity_north_mps, pose.velocity_east_mps
        if not position_enabled:
            self._flow_weight = 0.0
        if position_enabled and f.mode == "relative":
            if flow_log.get("flow_watchdog") == "lost":
                raise SafetyStop("Relative hold lost optical flow: " + str(flow_log.get("flow_reason")))
            pose = telemetry.local_pose
            assert pose is not None
            if self._local_pose_xy_origin is None:
                self._local_pose_xy_origin = (pose.north_m, pose.east_m)
            north = pose.north_m - self._local_pose_xy_origin[0]
            east = pose.east_m - self._local_pose_xy_origin[1]
            position_error = math.hypot(north, east)
            if position_error > self.config.safety.max_position_error_m:
                raise SafetyStop("Relative position exceeds safety envelope")
            flow_log.update(relative_north_m=north, relative_east_m=east)
        if position_enabled:
            # Outer P: position [m] -> velocity [m/s], vector speed limit.
            c = self.config.control
            vn_set = -c.position_velocity_kp * north
            ve_set = -c.position_velocity_kp * east
            speed_set = math.hypot(vn_set, ve_set)
            if speed_set > c.horizontal_speed_limit_mps:
                scale = c.horizontal_speed_limit_mps / speed_set
                vn_set *= scale
                ve_set *= scale
            if f.mode == "relative":
                vn, ve = flow_n, flow_e
                flow_log.update(flow_weight=1.0, velocity_source="INAV_LOCAL_POSE")
            else:
                # Course over ground is not aircraft yaw. Use measured GNSS speed,
                # avoiding differentiation of repeated/stale latitude samples.
                speed = telemetry.gps.ground_speed_mps
                course = telemetry.gps.ground_course_deg
                if not math.isfinite(speed) or speed < 0 or not math.isfinite(course):
                    raise SafetyStop("Invalid GPS horizontal velocity")
                vn = speed * math.cos(math.radians(course))
                ve = speed * math.sin(math.radians(course))
                flow_log.update(gps_velocity_north_mps=vn, gps_velocity_east_mps=ve)
                desired_weight = f.weight if f.mode == "assist" and flow_log.get("flow_valid") else 0.0
                old_weight = getattr(self, "_flow_weight", 0.0)
                # Never reuse invalid/stale velocity. Re-enter assistance gradually.
                weight = min(desired_weight, old_weight + dt * f.weight / f.transition_s)
                if desired_weight == 0 and old_weight > 0:
                    self.north_pid.reset()
                    self.east_pid.reset()
                self._flow_weight = weight
                if weight > 0:
                    vn = (1 - weight) * vn + weight * flow_n
                    ve = (1 - weight) * ve + weight * flow_e
                flow_log.update(flow_weight=weight, velocity_source="GPS_FLOW" if weight else "GPS")
            # Inner PI: velocity error [m/s] -> tilt [deg]. No derivative kick.
            old_n, old_e = self.north_pid.integral, self.east_pid.integral
            north_terms = self.north_pid.update(vn_set, vn, dt)
            east_terms = self.east_pid.update(ve_set, ve, dt)
            velocity_log = {
                "horizontal_control_mode": "POSITION_P_VELOCITY_PI",
                "velocity_north_setpoint_mps": vn_set,
                "velocity_east_setpoint_mps": ve_set,
                "velocity_north_mps": vn, "velocity_east_mps": ve,
                "velocity_north_error_mps": north_terms.error,
                "velocity_east_error_mps": east_terms.error,
            }
            gps_north_error = -north
            gps_north_p = north_terms.p
            gps_north_i = north_terms.i
            gps_north_d = north_terms.d
            gps_north_output = north_terms.output
            gps_east_error = -east
            gps_east_p = east_terms.p
            gps_east_i = east_terms.i
            gps_east_d = east_terms.d
            gps_east_output = east_terms.output
            tilt_north, tilt_east = north_terms.output, east_terms.output
            magnitude = math.hypot(tilt_north, tilt_east)
            tilt_limit = (
                self.config.control.max_tilt_deg
                if position_tilt_limit_deg is None
                else min(self.config.control.max_tilt_deg, position_tilt_limit_deg)
            )
            if magnitude > tilt_limit:
                # Do not accumulate I behind the downstream vector limiter.
                self.north_pid.integral = old_n
                self.east_pid.integral = old_e
                scale = tilt_limit / magnitude
                tilt_north *= scale
                tilt_east *= scale
            tilt_north_limited = tilt_north
            tilt_east_limited = tilt_east
            forward_tilt, right_tilt = ne_to_body(
                tilt_north, tilt_east, telemetry.attitude.yaw_deg
            )
            roll_pwm = round(
                self.config.control.rc_mid
                + self.config.control.roll_sign
                * right_tilt
                * self.config.control.pwm_per_degree
            )
            pitch_pwm = round(
                self.config.control.rc_mid
                + self.config.control.pitch_sign
                * forward_tilt
                * self.config.control.pwm_per_degree
            )
        else:
            self.north_pid.reset()
            self.east_pid.reset()
            # Altitude-only keeps physical roll/pitch/yaw and never uses GPS.
            roll_pwm = self.config.control.rc_mid
            pitch_pwm = self.config.control.rc_mid

        altitude_terms = self.altitude_pid.update(
            height_setpoint,
            telemetry.height_agl_m,
            dt,
            integrate=altitude_integrate,
        )
        desired_throttle = clamp(
            self.config.control.throttle_hover + altitude_terms.output,
            self.config.control.throttle_min_flight,
            self.config.control.throttle_max_flight,
        )
        max_step = self.config.control.throttle_slew_us_per_s * dt
        self.current_throttle += clamp(
            desired_throttle - self.current_throttle, -max_step, max_step
        )
        throttle_pwm = round(self.current_throttle)
        frame = self._frame(
            armed=True,
            throttle=throttle_pwm,
            roll=roll_pwm,
            pitch=pitch_pwm,
        )
        self._send(frame)
        self._check_control_permission()

        row: dict[str, object] = {
            "gps_sample_valid": int(gps_sample_valid),
            "gps_raw_north_m": gps_raw_north if gps_sample_valid else "",
            "gps_raw_east_m": gps_raw_east if gps_sample_valid else "",
            "position_source": ("NONE" if not position_enabled else
                                ("FLOW_RELATIVE" if f.mode == "relative" else "GPS")),
            **flow_log,
            **velocity_log,
            "mission_mode": "ALTITUDE_ONLY" if self.altitude_only else "FULL_GPS",
            "event": "",
            "active_modes": "|".join(sorted(self._last_active_modes)),
            "position_pid_enabled": int(position_enabled),
            "height_agl_m": telemetry.height_agl_m,
            "height_raw_m": telemetry.height_raw_m,
            "rangefinder_distance_m": telemetry.rangefinder_distance_m,
            "height_setpoint_m": height_setpoint,
            "vertical_speed_mps": telemetry.vertical_speed_mps,
            "inav_vertical_speed_mps": telemetry.inav_vertical_speed_mps,
            "gps_calib_lat": self.ground.lat if self.gps_enabled else "",
            "gps_calib_lon": self.ground.lon if self.gps_enabled else "",
            "gps_measurement_lat": telemetry.gps.lat if self.gps_enabled else "",
            "gps_measurement_lon": telemetry.gps.lon if self.gps_enabled else "",
            "gps_lat": telemetry.gps.lat if self.gps_enabled else "",
            "gps_lon": telemetry.gps.lon if self.gps_enabled else "",
            "north_m": north if self.gps_enabled else "",
            "east_m": east if self.gps_enabled else "",
            "position_error_m": position_error if self.gps_enabled else "",
            "gps_north_error_m": gps_north_error,
            "gps_north_p_deg": gps_north_p,
            "gps_north_i_deg": gps_north_i,
            "gps_north_d_deg": gps_north_d,
            "gps_north_output_deg": gps_north_output,
            "gps_east_error_m": gps_east_error,
            "gps_east_p_deg": gps_east_p,
            "gps_east_i_deg": gps_east_i,
            "gps_east_d_deg": gps_east_d,
            "gps_east_output_deg": gps_east_output,
            "tilt_north_limited_deg": tilt_north_limited,
            "tilt_east_limited_deg": tilt_east_limited,
            "forward_tilt_deg": forward_tilt,
            "right_tilt_deg": right_tilt,
            "roll_deg": telemetry.attitude.roll_deg,
            "pitch_deg": telemetry.attitude.pitch_deg,
            "yaw_deg": telemetry.attitude.yaw_deg,
            "roll_pwm": roll_pwm,
            "pitch_pwm": pitch_pwm,
            "throttle_pwm": throttle_pwm,
            "alt_p": altitude_terms.p,
            "alt_i": altitude_terms.i,
            "alt_d": altitude_terms.d,
            "gps_fix": telemetry.gps.fix_type if self.gps_enabled else "",
            "satellites": telemetry.gps.satellites if self.gps_enabled else "",
            "hdop": telemetry.gps.hdop if self.gps_enabled else "",
            "motor_1_output": "" if telemetry.motor_outputs[0] < 0 else telemetry.motor_outputs[0],
            "motor_2_output": "" if telemetry.motor_outputs[1] < 0 else telemetry.motor_outputs[1],
            "motor_3_output": "" if telemetry.motor_outputs[2] < 0 else telemetry.motor_outputs[2],
            "motor_4_output": "" if telemetry.motor_outputs[3] < 0 else telemetry.motor_outputs[3],
            "motor_1_rpm": "" if telemetry.motor_rpm[0] < 0 else telemetry.motor_rpm[0],
            "motor_2_rpm": "" if telemetry.motor_rpm[1] < 0 else telemetry.motor_rpm[1],
            "motor_3_rpm": "" if telemetry.motor_rpm[2] < 0 else telemetry.motor_rpm[2],
            "motor_4_rpm": "" if telemetry.motor_rpm[3] < 0 else telemetry.motor_rpm[3],
        }
        if self._log is not None:
            self._log.write(
                {
                    **row,
                    "time_s": telemetry.timestamp - self._mission_started,
                    "state": self.state.value,
                }
            )
        if telemetry.timestamp - self._last_console >= self.config.logging.console_period_s:
            output_text = format_motor_outputs(telemetry.motor_outputs)
            rpm_text = format_motor_rpm(telemetry.motor_rpm)
            gps_text = (
                ((f"FLOWpos={position_error:.2f} m "
                  f"GPS={telemetry.gps.lat:.7f},{telemetry.gps.lon:.7f}")
                 if self.relative_mode else
                 (f"GPS={telemetry.gps.lat:.7f},{telemetry.gps.lon:.7f} "
                  f"pos={position_error:.2f} m"))
                if self.gps_enabled else "GPS=disabled"
            )
            self.output(
                f"{self.state.value:7s} z={telemetry.height_agl_m:.2f}/{height_setpoint:.2f} m "
                f"vz={telemetry.vertical_speed_mps:+.2f} "
                f"(INAV={telemetry.inav_vertical_speed_mps:+.2f}) m/s {gps_text} "
                f"OUT[{output_text}] RPM[{rpm_text}] "
                f"RC={roll_pwm}/{pitch_pwm}/{throttle_pwm}"
            )
            self._last_console = telemetry.timestamp
        return row

    def _reset_pids(self, telemetry: Telemetry) -> None:
        assert self.ground is not None
        if self.relative_mode:
            north = east = 0.0
        elif self.gps_enabled:
            north, east = local_ne_m(
                self.ground.lat,
                self.ground.lon,
                telemetry.gps.lat,
                telemetry.gps.lon,
            )
        else:
            north = east = 0.0
        self.altitude_pid.reset(telemetry.height_agl_m)
        self.north_pid.reset(north)
        self.east_pid.reset(east)

    def takeoff(self) -> None:
        self._set_state(MissionState.TAKEOFF)
        first = self._telemetry()
        self._reset_pids(first)
        start = self.clock.monotonic()
        deadline = start + self.config.control.takeoff_timeout_s
        stable_since: float | None = None
        liftoff_since: float | None = None
        liftoff_confirmed = False
        period = 1.0 / self.config.control.loop_hz
        previous = start
        while self.clock.monotonic() < deadline:
            loop_start = self.clock.monotonic()
            dt = max(0.001, loop_start - previous)
            previous = loop_start
            elapsed = loop_start - start
            setpoint = min(
                self.config.control.target_altitude_m,
                elapsed * self.config.control.climb_rate_mps,
            )
            telemetry = self._telemetry()
            if telemetry.height_agl_m >= self.config.control.liftoff_detect_margin_m:
                liftoff_since = liftoff_since or loop_start
                if (
                    not liftoff_confirmed
                    and loop_start - liftoff_since
                    >= self.config.control.liftoff_confirm_time_s
                ):
                    liftoff_confirmed = True
                    relative = self.config.optical_flow.mode == "relative"
                    xy_text = (
                        (" Optical-flow relative X/Y hold enabled with takeoff tilt limit."
                         if relative else " GPS X/Y hold enabled with takeoff tilt limit.")
                        if self.gps_enabled
                        else " X/Y hold unavailable because GPS is disabled."
                    )
                    self.output(
                        f"Liftoff confirmed at z={telemetry.height_agl_m:.2f} m."
                        + xy_text
                    )
            else:
                liftoff_since = None
            if (
                not liftoff_confirmed
                and elapsed >= self.config.control.liftoff_timeout_s
            ):
                raise SafetyStop(
                    "No liftoff detected before liftoff timeout; refusing to keep "
                    "increasing throttle"
                )
            self._control_step(
                telemetry,
                setpoint,
                dt,
                position_enabled=self.gps_enabled and liftoff_confirmed,
                altitude_integrate=liftoff_confirmed,
                position_tilt_limit_deg=self.config.control.takeoff_position_max_tilt_deg,
            )
            stable = (
                liftoff_confirmed
                and
                setpoint >= self.config.control.target_altitude_m
                and abs(telemetry.height_agl_m - setpoint) <= self.config.control.altitude_tolerance_m
                and abs(telemetry.vertical_speed_mps) <= 0.25
            )
            if stable:
                stable_since = stable_since or loop_start
                if loop_start - stable_since >= self.config.control.stable_time_s:
                    position_text = (
                        ("Optical-flow relative X/Y hold remains active."
                         if self.config.optical_flow.mode == "relative"
                         else "GPS X/Y hold remains active.")
                        if self.gps_enabled
                        else "X/Y hold remains unavailable in altitude-only mode."
                    )
                    self.output("Takeoff altitude is stable; " + position_text)
                    return
            else:
                stable_since = None
            self.clock.sleep(max(0.0, period - (self.clock.monotonic() - loop_start)))
        raise SafetyStop("Takeoff timeout before altitude became stable")

    def hover_altitude(self) -> None:
        """Hold altitude for 10 s; full-GPS also holds the calibrated X/Y point."""
        self._set_state(MissionState.ALTITUDE_HOVER)
        start = self.clock.monotonic()
        deadline = start + self.config.control.altitude_hover_time_s
        previous = start
        period = 1.0 / self.config.control.loop_hz
        while self.clock.monotonic() < deadline:
            loop_start = self.clock.monotonic()
            dt = max(0.001, loop_start - previous)
            previous = loop_start
            telemetry = self._telemetry()
            self._control_step(
                telemetry,
                self.config.control.target_altitude_m,
                dt,
                position_enabled=self.gps_enabled,
                position_tilt_limit_deg=self.config.control.takeoff_position_max_tilt_deg,
            )
            self.clock.sleep(max(0.0, period - (self.clock.monotonic() - loop_start)))
        next_action = (
            "X/Y hold remains unavailable because GPS is disabled."
            if self.altitude_only
            else (("Relative X/Y hold remained active; entering position verification now."
                   if self.config.optical_flow.mode == "relative"
                   else "GPS X/Y hold remained active; entering position verification now."))
        )
        self.output(
            f"Altitude held for {self.config.control.altitude_hover_time_s:.1f} s. "
            + next_action
        )

    def acquire_gps_position(self) -> None:
        """Enable GPS PID only now and settle back onto the calibrated ground point."""
        self._set_state(MissionState.GPS_ACQUIRE)
        first = self._telemetry()
        assert self.ground is not None
        if self.relative_mode:
            north = east = 0.0
        else:
            north, east = local_ne_m(
                self.ground.lat,
                self.ground.lon,
                first.gps.lat,
                first.gps.lon,
            )
        # Reset at the switchover so the first GPS derivative term is zero.
        self.north_pid.reset(north)
        self.east_pid.reset(east)
        start = self.clock.monotonic()
        deadline = start + self.config.control.gps_acquire_timeout_s
        previous = start
        stable_since: float | None = None
        period = 1.0 / self.config.control.loop_hz
        while self.clock.monotonic() < deadline:
            loop_start = self.clock.monotonic()
            dt = max(0.001, loop_start - previous)
            previous = loop_start
            telemetry = self._telemetry()
            row = self._control_step(
                telemetry,
                self.config.control.target_altitude_m,
                dt,
                position_enabled=True,
            )
            stable = (
                float(row["position_error_m"]) <= self.config.control.position_tolerance_m
                and abs(
                    telemetry.height_agl_m - self.config.control.target_altitude_m
                )
                <= self.config.control.altitude_tolerance_m
                and abs(telemetry.vertical_speed_mps) <= 0.25
            )
            if stable:
                stable_since = stable_since or loop_start
                if loop_start - stable_since >= self.config.control.stable_time_s:
                    source = "Relative optical-flow target" if self.config.optical_flow.mode == "relative" else "GPS target"
                    self.output(source + " acquired and stable.")
                    return
            else:
                stable_since = None
            self.clock.sleep(max(0.0, period - (self.clock.monotonic() - loop_start)))
        raise SafetyStop("Horizontal target did not become stable before acquire timeout")

    def hold_gps_position(self) -> None:
        self._set_state(MissionState.GPS_HOLD)
        start = self.clock.monotonic()
        deadline = start + self.config.control.gps_hold_time_s
        previous = start
        period = 1.0 / self.config.control.loop_hz
        while self.clock.monotonic() < deadline:
            loop_start = self.clock.monotonic()
            dt = max(0.001, loop_start - previous)
            previous = loop_start
            telemetry = self._telemetry()
            self._control_step(
                telemetry,
                self.config.control.target_altitude_m,
                dt,
                position_enabled=True,
            )
            self.clock.sleep(max(0.0, period - (self.clock.monotonic() - loop_start)))
        source = "Relative position" if self.config.optical_flow.mode == "relative" else "GPS position"
        self.output(f"{source} and altitude held for {self.config.control.gps_hold_time_s:.1f} s.")

    def land(self, *, position_enabled: bool = True) -> None:
        self._set_state(MissionState.LAND)
        start_telemetry = self._telemetry()
        start_height = start_telemetry.height_agl_m
        start = self.clock.monotonic()
        deadline = start + self.config.control.landing_timeout_s
        previous = start
        period = 1.0 / self.config.control.loop_hz
        ground_since: float | None = None
        while self.clock.monotonic() < deadline:
            loop_start = self.clock.monotonic()
            dt = max(0.001, loop_start - previous)
            previous = loop_start
            setpoint = max(
                0.0,
                start_height - self.config.control.descent_rate_mps * (loop_start - start),
            )
            telemetry = self._telemetry()
            self._control_step(
                telemetry,
                setpoint,
                dt,
                position_enabled=position_enabled,
            )
            if (
                telemetry.height_agl_m <= self.config.control.ground_detect_margin_m
                and abs(telemetry.vertical_speed_mps) <= 0.20
            ):
                ground_since = ground_since or loop_start
                if loop_start - ground_since >= self.config.control.ground_confirm_time_s:
                    self.output("Ground confirmed from fresh height and vertical-speed samples.")
                    break
            else:
                ground_since = None
            self.clock.sleep(max(0.0, period - (self.clock.monotonic() - loop_start)))
        else:
            raise SafetyStop("Landing timeout before ground confirmation")

        while self.current_throttle > self.config.control.rc_min:
            started = self.clock.monotonic()
            self.current_throttle = max(
                self.config.control.rc_min,
                self.current_throttle - self.config.control.throttle_slew_us_per_s * period,
            )
            self._send(self._frame(armed=True, throttle=round(self.current_throttle)))
            self._check_control_permission()
            self.clock.sleep(max(0.0, period - (self.clock.monotonic() - started)))

    def disarm(self) -> None:
        self._set_state(MissionState.DISARM)
        frame = self._frame(armed=False, throttle=self.config.control.rc_min)
        self._stream(1.0, frame)
        if "ARM" in self.vehicle.active_modes():
            raise SafetyStop("INAV still reports ARM after disarm command")
        self._armed = False
        self.output("INAV confirmed DISARMED.")

    def _announce_real_flight(self) -> None:
        if self.simulation:
            return
        if self.bench_arm_test_enabled:
            mode_warning = (
                "BENCH ARM TEST: physically remove all propellers. Pi will auto-ARM "
                "at minimum throttle and auto-DISARM. "
            )
        elif self.altitude_only:
            mode_warning = (
                "ALTITUDE-ONLY: Pi will auto-ARM and control throttle; keep physical "
                "ANGLE ON and roll/pitch/yaw centered. "
            )
        else:
            mode_warning = ""
        self.output(
            mode_warning + "Clear the area, verify props/RC failsafe/takeover. "
            "No typed confirmation required; waiting for physical CH8 takeover."
        )

    def run(self) -> Path | None:
        try:
            self.preflight()
            self.calibrate_ground()
            if not self.fly and not self.simulation:
                self.output("\n[CHECK PASS] No MSP RC, ARM, or motor command was sent.")
                return None
            self._announce_real_flight()
            self.wait_for_takeover()
            self.verify_takeover()
            self._mission_started = self.clock.monotonic()
            log_dir = self.log_root / self.config.logging.directory
            self._log = FlightLog(log_dir, self._mission_started)
            self.arm()
            if self.bench_arm_test_enabled:
                self.bench_arm_test()
                self.disarm()
                self._set_state(MissionState.COMPLETE)
                if self._log:
                    self.output(f"Bench log: {self._log.path}")
                    return self._log.path
                return None
            self.takeoff()
            self.hover_altitude()
            if self.altitude_only:
                self.output(
                    "Altitude-only hover complete; GPS PID remains OFF. Landing now."
                )
                self.land(position_enabled=False)
            else:
                self.acquire_gps_position()
                self.hold_gps_position()
                self.land(position_enabled=True)
            self.disarm()
            self._set_state(MissionState.COMPLETE)
            if self._log:
                self.output(f"Flight log: {self._log.path}")
                return self._log.path
            return None
        except ManualTakeover as exc:
            self.state = MissionState.MANUAL
            if self._log is not None:
                self._log.write(
                    {
                        "time_s": self.clock.monotonic() - self._mission_started,
                        "mission_mode": "ALTITUDE_ONLY" if self.altitude_only else "FULL_GPS",
                        "state": self.state.value,
                        "event": f"MANUAL TAKEOVER: {exc}",
                        "active_modes": "|".join(sorted(self._last_active_modes)),
                    }
                )
            raise
        except Exception as exc:
            self.state = MissionState.ABORT
            if self._log is not None:
                self._log.write(
                    {
                        "time_s": self.clock.monotonic() - self._mission_started,
                        "mission_mode": "ALTITUDE_ONLY" if self.altitude_only else "FULL_GPS",
                        "state": self.state.value,
                        "event": f"{type(exc).__name__}: {exc}",
                        "active_modes": "|".join(sorted(self._last_active_modes)),
                    }
                )
            raise
        finally:
            if self._log is not None:
                self._log.close()

    @property
    def armed(self) -> bool:
        return self._armed
