from __future__ import annotations

from dataclasses import dataclass
import math


HW_SENSOR_NONE = 0
HW_SENSOR_OK = 1
HW_SENSOR_UNAVAILABLE = 2
HW_SENSOR_UNHEALTHY = 3

GPS_FIX_3D = 2


@dataclass(frozen=True)
class APIVersion:
    protocol: int
    major: int
    minor: int


@dataclass(frozen=True)
class GPSData:
    fix_type: int
    satellites: int
    lat: float
    lon: float
    hdop: float
    ground_speed_mps: float = 0.0
    ground_course_deg: float = 0.0


@dataclass(frozen=True)
class AltitudeData:
    estimated_alt_m: float
    vertical_speed_mps: float
    barometer_alt_m: float


@dataclass(frozen=True)
class AttitudeData:
    roll_deg: float
    pitch_deg: float
    yaw_deg: float


@dataclass(frozen=True)
class LocalPoseData:
    """INAV earth-frame estimator output (metres and metres/second)."""
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    north_m: float
    east_m: float
    up_m: float
    velocity_north_mps: float
    velocity_east_mps: float
    velocity_up_mps: float


@dataclass(frozen=True)
class SensorStatus:
    overall_healthy: bool
    gyro: int
    accelerometer: int
    compass: int
    barometer: int
    gps: int
    rangefinder: int
    pitot: int
    optical_flow: int


@dataclass(frozen=True)
class ModeRange:
    permanent_id: int
    name: str
    aux_index: int
    start_step: int
    end_step: int

    @property
    def channel_index(self) -> int:
        return 4 + self.aux_index

    @property
    def start_pwm(self) -> int:
        return 900 + 25 * self.start_step

    @property
    def end_pwm(self) -> int:
        return 900 + 25 * self.end_step

    def contains(self, pwm: int) -> bool:
        step = (max(900, min(2100, int(pwm))) - 900) // 25
        return self.start_step <= step < self.end_step


@dataclass(frozen=True)
class GroundReference:
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    lat: float
    lon: float
    height_raw_m: float
    rangefinder_distance_m: float = 0.0


@dataclass(frozen=True)
class Telemetry:
    timestamp: float
    attitude: AttitudeData
    gps: GPSData
    height_raw_m: float
    height_agl_m: float
    vertical_speed_mps: float
    inav_vertical_speed_mps: float
    motor_outputs: tuple[int, int, int, int]
    motor_rpm: tuple[int, int, int, int]
    rangefinder_distance_m: float = math.nan
    local_pose: LocalPoseData | None = None
