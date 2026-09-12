from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping


class ConfigError(ValueError):
    pass


@dataclass(frozen=True)
class PIDConfig:
    kp: float
    ki: float
    kd: float
    output_limit: float
    integrator_limit: float
    derivative_filter_hz: float = 5.0


@dataclass(frozen=True)
class SerialConfig:
    port: str = "/dev/serial0"
    baud: int = 115200
    request_timeout_s: float = 0.18
    retries: int = 1
    startup_timeout_s: float = 5.0
    startup_retry_delay_s: float = 0.25


@dataclass(frozen=True)
class SensorConfig:
    height_source: str = "rangefinder"
    min_range_m: float = 0.02
    max_range_m: float = 3.5
    calibration_samples: int = 40
    calibration_hz: float = 10.0
    max_ground_tilt_deg: float = 8.0
    max_height_std_m: float = 0.03
    max_position_std_m: float = 1.5
    min_satellites: int = 8
    max_hdop: float = 2.5
    height_velocity_filter_hz: float = 2.0


@dataclass(frozen=True)
class ControlConfig:
    loop_hz: float = 10.0
    target_altitude_m: float = 1.5
    altitude_hover_time_s: float = 10.0
    gps_acquire_timeout_s: float = 20.0
    gps_hold_time_s: float = 10.0
    climb_rate_mps: float = 0.30
    descent_rate_mps: float = 0.18
    takeoff_timeout_s: float = 25.0
    landing_timeout_s: float = 25.0
    stable_time_s: float = 1.5
    altitude_tolerance_m: float = 0.12
    position_tolerance_m: float = 1.8
    ground_detect_margin_m: float = 0.08
    ground_confirm_time_s: float = 1.0
    liftoff_detect_margin_m: float = 0.12
    liftoff_confirm_time_s: float = 0.30
    liftoff_timeout_s: float = 8.0
    rc_min: int = 1000
    rc_mid: int = 1500
    throttle_hover: int = 1500
    throttle_min_flight: int = 1300
    throttle_max_flight: int = 1700
    throttle_slew_us_per_s: float = 120.0
    max_tilt_deg: float = 10.0
    takeoff_position_max_tilt_deg: float = 4.0
    pwm_per_degree: float = 18.0
    roll_sign: int = 1
    pitch_sign: int = -1
    position_velocity_kp: float = 0.5
    horizontal_speed_limit_mps: float = 0.5
    velocity_tilt_kp: float = 5.0
    velocity_tilt_ki: float = 0.3
    altitude_pid: PIDConfig = field(
        default_factory=lambda: PIDConfig(160.0, 35.0, 65.0, 170.0, 90.0, 6.0)
    )
    position_north_pid: PIDConfig = field(
        default_factory=lambda: PIDConfig(1.4, 0.08, 0.65, 10.0, 2.5, 2.0)
    )
    position_east_pid: PIDConfig = field(
        default_factory=lambda: PIDConfig(1.4, 0.08, 0.65, 10.0, 2.5, 2.0)
    )


@dataclass(frozen=True)
class SafetyConfig:
    takeover_channel: int = 8
    takeover_off_pwm: int = 1500
    takeover_on_pwm: int = 1700
    takeover_wait_timeout_s: float = 60.0
    arm_timeout_s: float = 6.0
    override_settle_s: float = 3.0
    max_position_error_m: float = 8.0
    max_flight_tilt_deg: float = 30.0
    max_vertical_speed_mps: float = 2.0
    max_control_gap_s: float = 0.5
    maximum_mission_time_s: float = 90.0
    required_confirmation: str = "ARM AND FLY"
    altitude_only_confirmation: str = "ARM ALTITUDE ONLY"
    bench_confirmation: str = "PROPS REMOVED"
    bench_arm_hold_s: float = 3.0
    manual_stick_center_tolerance_pwm: int = 80


@dataclass(frozen=True)
class LoggingConfig:
    directory: str = "logs"
    console_period_s: float = 0.5
    motor_rpm_hz: float = 2.0
    require_motor_rpm: bool = True


@dataclass(frozen=True)
class OpticalFlowConfig:
    mode: str = "off"  # off, monitor, assist, relative
    calibrated: bool = False
    forward_axis: str = "y"
    right_axis: str = "x"
    forward_sign: int = -1
    right_sign: int = 1
    scale: float = 1.0
    min_quality: int = 70
    min_range_m: float = 0.25
    max_range_m: float = 2.5
    max_tilt_deg: float = 12.0
    max_speed_mps: float = 2.0
    weight: float = 0.7
    transition_s: float = 1.0
    flow_loss_timeout_s: float = 0.4
    local_pose_max_speed_mps: float = 3.0


@dataclass(frozen=True)
class AppConfig:
    serial: SerialConfig = field(default_factory=SerialConfig)
    sensors: SensorConfig = field(default_factory=SensorConfig)
    control: ControlConfig = field(default_factory=ControlConfig)
    safety: SafetyConfig = field(default_factory=SafetyConfig)
    logging: LoggingConfig = field(default_factory=LoggingConfig)
    optical_flow: OpticalFlowConfig = field(default_factory=lambda: OpticalFlowConfig())


def _section(cls: type, source: Mapping[str, Any], name: str):
    raw = source.get(name, {})
    if not isinstance(raw, Mapping):
        raise ConfigError(f"'{name}' must be a JSON object")
    try:
        return cls(**raw)
    except TypeError as exc:
        raise ConfigError(f"Invalid '{name}' configuration: {exc}") from exc


def _pid(raw: Mapping[str, Any], key: str, default: PIDConfig) -> PIDConfig:
    value = raw.get(key)
    if value is None:
        return default
    if not isinstance(value, Mapping):
        raise ConfigError(f"control.{key} must be a JSON object")
    try:
        return PIDConfig(**value)
    except TypeError as exc:
        raise ConfigError(f"Invalid control.{key}: {exc}") from exc


def load_config(path: str | Path) -> AppConfig:
    config_path = Path(path)
    try:
        raw = json.loads(config_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ConfigError(f"Cannot read configuration {config_path}: {exc}") from exc
    if not isinstance(raw, Mapping):
        raise ConfigError("Top-level configuration must be a JSON object")

    defaults = AppConfig()
    control_raw = raw.get("control", {})
    if not isinstance(control_raw, Mapping):
        raise ConfigError("'control' must be a JSON object")
    control_values = dict(control_raw)
    for name, upper in (("position_velocity_kp", 5.0),
                        ("horizontal_speed_limit_mps", 2.0),
                        ("velocity_tilt_kp", 30.0), ("velocity_tilt_ki", 5.0)):
        value = control_values.get(name, getattr(defaults.control, name))
        if not isinstance(value, (int, float)) or not 0 < value <= upper:
            raise ConfigError(f"control.{name} must be positive and <= {upper}")
    control_values["altitude_pid"] = _pid(
        control_raw, "altitude_pid", defaults.control.altitude_pid
    )
    control_values["position_north_pid"] = _pid(
        control_raw, "position_north_pid", defaults.control.position_north_pid
    )
    control_values["position_east_pid"] = _pid(
        control_raw, "position_east_pid", defaults.control.position_east_pid
    )
    try:
        control = ControlConfig(**control_values)
    except TypeError as exc:
        raise ConfigError(f"Invalid 'control' configuration: {exc}") from exc

    config = AppConfig(
        serial=_section(SerialConfig, raw, "serial"),
        sensors=_section(SensorConfig, raw, "sensors"),
        control=control,
        safety=_section(SafetyConfig, raw, "safety"),
        logging=_section(LoggingConfig, raw, "logging"),
        optical_flow=_section(OpticalFlowConfig, raw, "optical_flow"),
    )
    validate_config(config)
    return config


def validate_config(config: AppConfig) -> None:
    f = config.optical_flow
    if f.mode not in {"off", "monitor", "assist", "relative"}:
        raise ConfigError("optical_flow.mode must be off, monitor, assist or relative")
    if f.mode in {"assist", "relative"} and f.calibrated is not True:
        raise ConfigError("Optical flow assist requires verified axes/scale and calibrated=true")
    if {f.forward_axis, f.right_axis} != {"x", "y"} or f.forward_sign not in {-1, 1} or f.right_sign not in {-1, 1}:
        raise ConfigError("Optical flow requires distinct x/y axes and signs +/-1")
    if not (0.1 <= f.scale <= 10 and 1 <= f.min_quality <= 100
            and 0.02 <= f.min_range_m < f.max_range_m <= 3.5
            and 1 <= f.max_tilt_deg <= 20 and 0 < f.max_speed_mps <= 5
            and 0 < f.weight <= 1 and 0.5 <= f.transition_s <= 5):
        raise ConfigError("Invalid optical flow limits/scale/weight")
    if f.mode != "off" and config.sensors.height_source != "rangefinder":
        raise ConfigError("Optical flow requires rangefinder height source")
    if not (0.2 <= f.flow_loss_timeout_s <= 1.0
            and 0.5 <= f.local_pose_max_speed_mps <= 5.0):
        raise ConfigError("Invalid relative-position watchdog/local-pose limits")
    c, s, safe = config.control, config.sensors, config.safety
    if not 1.0 <= config.serial.startup_timeout_s <= 15.0:
        raise ConfigError("serial.startup_timeout_s must be within 1..15 s")
    if not 0.05 <= config.serial.startup_retry_delay_s <= 1.0:
        raise ConfigError("serial.startup_retry_delay_s must be within 0.05..1.0 s")
    if s.height_source not in {"rangefinder", "barometer"}:
        raise ConfigError("sensors.height_source must be 'rangefinder' or 'barometer'")
    if not 0.2 <= s.height_velocity_filter_hz <= 10.0:
        raise ConfigError("sensors.height_velocity_filter_hz must be within 0.2..10 Hz")
    if not 5.0 <= c.loop_hz <= 20.0:
        raise ConfigError("control.loop_hz must be within 5..20 Hz")
    if not 0.3 <= c.target_altitude_m <= min(10.0, s.max_range_m - 0.1 if s.height_source == "rangefinder" else 10.0):
        raise ConfigError("target_altitude_m is outside the allowed sensor/flight envelope")
    if (
        c.altitude_hover_time_s <= 0
        or c.gps_acquire_timeout_s <= 0
        or c.gps_hold_time_s <= 0
        or c.climb_rate_mps <= 0
        or c.descent_rate_mps <= 0
    ):
        raise ConfigError("hover/acquire/hold times and climb/descent rates must be positive")
    if not c.rc_min < c.throttle_min_flight < c.throttle_hover < c.throttle_max_flight <= 2000:
        raise ConfigError("Require rc_min < throttle_min_flight < hover < max <= 2000")
    if not c.ground_detect_margin_m < c.liftoff_detect_margin_m < c.target_altitude_m:
        raise ConfigError(
            "Require ground_detect_margin_m < liftoff_detect_margin_m < target_altitude_m"
        )
    if not 0.1 <= c.liftoff_confirm_time_s <= 2.0:
        raise ConfigError("control.liftoff_confirm_time_s must be within 0.1..2 s")
    if not 1.0 <= c.liftoff_timeout_s < c.takeoff_timeout_s:
        raise ConfigError("control.liftoff_timeout_s must be below takeoff_timeout_s")
    if not 1.0 <= c.takeoff_position_max_tilt_deg <= c.max_tilt_deg:
        raise ConfigError(
            "control.takeoff_position_max_tilt_deg must be within 1..max_tilt_deg"
        )
    if c.roll_sign not in {-1, 1} or c.pitch_sign not in {-1, 1}:
        raise ConfigError("roll_sign and pitch_sign must be -1 or 1")
    if not 5 <= safe.takeover_channel <= 18:
        raise ConfigError("takeover_channel must be an AUX channel (5..18)")
    if safe.takeover_off_pwm >= safe.takeover_on_pwm:
        raise ConfigError("takeover_off_pwm must be below takeover_on_pwm")
    if not 0.2 <= safe.max_control_gap_s <= 1.0:
        raise ConfigError("safety.max_control_gap_s must be within 0.2..1.0 s")
    if not 1.0 <= safe.override_settle_s <= 5.0:
        raise ConfigError("safety.override_settle_s must be within 1..5 s")
    if not 30 <= safe.manual_stick_center_tolerance_pwm <= 150:
        raise ConfigError(
            "safety.manual_stick_center_tolerance_pwm must be within 30..150"
        )
    if not 1.0 <= safe.bench_arm_hold_s <= 10.0:
        raise ConfigError("safety.bench_arm_hold_s must be within 1..10 s")
    if not 0.5 <= config.logging.motor_rpm_hz <= 5.0:
        raise ConfigError("logging.motor_rpm_hz must be within 0.5..5.0 Hz")
    for name, pid in (
        ("altitude_pid", c.altitude_pid),
        ("position_north_pid", c.position_north_pid),
        ("position_east_pid", c.position_east_pid),
    ):
        if min(pid.kp, pid.ki, pid.kd, pid.output_limit, pid.integrator_limit) < 0:
            raise ConfigError(f"control.{name} values cannot be negative")
