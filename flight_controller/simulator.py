from __future__ import annotations

import math
from typing import Sequence

from .config import AppConfig
from .models import (
    APIVersion,
    AltitudeData,
    AttitudeData,
    GPSData,
    LocalPoseData,
    ModeRange,
    SensorStatus,
)


class SimulationVehicle:
    """Small deterministic plant used to exercise the complete mission offline."""

    def __init__(self, config: AppConfig, *, altitude_only: bool = False) -> None:
        self.config = config
        self.time = 0.0
        self.origin_lat = 10.7622403
        self.origin_lon = 106.6812774
        self.mount_height_m = 0.12
        self.north_m = 0.8
        self.east_m = -0.5
        self.vn = 0.0
        self.ve = 0.0
        self.height_m = 0.0
        self.vz = 0.0
        self.yaw_deg = 32.0
        self.wind_accel_north = 0.09
        self.wind_accel_east = -0.06
        self.roll_deg = 0.5
        self.pitch_deg = -0.4
        self._armed = False
        self._physical_takeover = False
        self._rc_reads = 0
        self._channels: list[int] | None = None
        self._override_indices = {3, 4} if altitude_only else set(range(6))
        self._physical_channels = [1500] * 12
        self._physical_channels[3] = 1000
        self._physical_channels[4] = 1000
        self._physical_channels[5] = 2000
        self._ranges = [
            ModeRange(0, "ARM", 0, 32, 48),
            ModeRange(1, "ANGLE", 1, 32, 48),
            ModeRange(50, "MSP RC OVERRIDE", 3, 32, 48),
        ]

    def api_version(self) -> APIVersion:
        return APIVersion(0, 2, 5)

    def fc_variant(self) -> str:
        return "INAV"

    def fc_version(self) -> tuple[int, int, int]:
        return 9, 0, 1

    def sensor_status(self) -> SensorStatus:
        return SensorStatus(True, 1, 1, 1, 1, 1, 1, 0, 1)

    def optical_flow(self):
        # Idealized aligned sensor; tests do not validate real mounting/gyro lag.
        f = self.config.optical_flow
        yaw = math.radians(self.yaw_deg)
        forward = self.vn * math.cos(yaw) + self.ve * math.sin(yaw)
        right = -self.vn * math.sin(yaw) + self.ve * math.cos(yaw)
        distance = self.rangefinder_m()
        rates = {}
        rates[f.forward_axis] = round(math.degrees(forward / (distance * f.scale * f.forward_sign)))
        rates[f.right_axis] = round(math.degrees(right / (distance * f.scale * f.right_sign)))
        return 100, rates['x'], rates['y'], 0, 0

    def local_pose(self) -> LocalPoseData:
        return LocalPoseData(
            self.roll_deg, self.pitch_deg, self.yaw_deg,
            self.north_m, self.east_m, self.height_m,
            self.vn, self.ve, self.vz,
        )

    def gps(self) -> GPSData:
        lat = self.origin_lat + math.degrees(self.north_m / 6_371_008.8)
        lon = self.origin_lon + math.degrees(
            self.east_m / (6_371_008.8 * math.cos(math.radians(self.origin_lat)))
        )
        return GPSData(2, 12, lat, lon, 1.1, math.hypot(self.vn, self.ve),
                       math.degrees(math.atan2(self.ve, self.vn)) % 360.0)

    def altitude(self) -> AltitudeData:
        return AltitudeData(self.height_m, self.vz, 100.0 + self.height_m)

    def rangefinder_m(self) -> float:
        return self.mount_height_m + self.height_m

    def attitude(self) -> AttitudeData:
        return AttitudeData(self.roll_deg, self.pitch_deg, self.yaw_deg)

    def rc_channels(self) -> list[int]:
        self._rc_reads += 1
        if self._rc_reads >= 3:
            self._physical_takeover = True
        values = list(self._physical_channels)
        values[7] = 2000 if self._physical_takeover else 1000
        if self._channels is not None and self._physical_takeover:
            for index in self._override_indices:
                if index < min(len(values), len(self._channels)):
                    values[index] = self._channels[index]
        return values

    def rx_map(self) -> list[int]:
        return [0, 1, 2, 3]

    def mode_ranges(self) -> list[ModeRange]:
        return list(self._ranges)

    def active_modes(self) -> set[str]:
        modes: set[str] = set()
        if self._physical_takeover:
            modes.add("MSP RC OVERRIDE")
        if self._physical_takeover:
            effective = self.rc_channels()
            for item in self._ranges:
                if item.name != "MSP RC OVERRIDE" and item.contains(effective[item.channel_index]):
                    modes.add(item.name)
        if self._armed:
            modes.add("ARM")
        else:
            modes.discard("ARM")
        return modes

    def motor_rpm(self) -> list[int]:
        if not self._armed or self._channels is None:
            return [0, 0, 0, 0]
        base = max(0, (self._channels[3] - self.config.control.rc_min) * 20)
        roll_mix = self._channels[0] - self.config.control.rc_mid
        pitch_mix = self._channels[1] - self.config.control.rc_mid
        return [
            max(0, base + roll_mix + pitch_mix),
            max(0, base - roll_mix + pitch_mix),
            max(0, base - roll_mix - pitch_mix),
            max(0, base + roll_mix - pitch_mix),
        ]

    def motor_outputs(self) -> list[int]:
        if not self._armed or self._channels is None:
            return [self.config.control.rc_min] * 4
        throttle = self._channels[3]
        roll_mix = self._channels[0] - self.config.control.rc_mid
        pitch_mix = self._channels[1] - self.config.control.rc_mid
        limit = lambda value: max(1000, min(2000, round(value)))
        return [
            limit(throttle + roll_mix + pitch_mix),
            limit(throttle - roll_mix + pitch_mix),
            limit(throttle - roll_mix - pitch_mix),
            limit(throttle + roll_mix - pitch_mix),
        ]

    def set_raw_rc(self, channels: Sequence[int], _rx_map: Sequence[int]) -> None:
        self._channels = [int(item) for item in channels]
        if not self._physical_takeover:
            return
        arm_range = self._ranges[0]
        effective = self.rc_channels()
        wants_arm = arm_range.contains(effective[arm_range.channel_index])
        throttle = effective[3]
        if wants_arm and throttle <= 1050:
            self._armed = True
        elif not wants_arm:
            self._armed = False

    def advance(self, dt: float) -> None:
        if dt <= 0:
            return
        steps = max(1, math.ceil(dt / 0.01))
        h = dt / steps
        for _ in range(steps):
            self._advance_step(h)
        self.time += dt

    def _advance_step(self, dt: float) -> None:
        if not self._armed or self._channels is None:
            self.height_m = 0.0
            self.vz = 0.0
            self.vn *= max(0.0, 1.0 - 3.0 * dt)
            self.ve *= max(0.0, 1.0 - 3.0 * dt)
            return

        c = self.config.control
        throttle = self._channels[3]
        vertical_accel = (throttle - c.throttle_hover) * 0.010 - 1.3 * self.vz
        self.vz += vertical_accel * dt
        self.height_m += self.vz * dt
        if self.height_m <= 0.0:
            self.height_m = 0.0
            self.vz = max(0.0, self.vz)

        desired_right = (self._channels[0] - c.rc_mid) / (c.roll_sign * c.pwm_per_degree)
        desired_forward = (self._channels[1] - c.rc_mid) / (c.pitch_sign * c.pwm_per_degree)
        self.roll_deg += (desired_right - self.roll_deg) * min(1.0, 5.0 * dt)
        self.pitch_deg += (desired_forward - self.pitch_deg) * min(1.0, 5.0 * dt)

        yaw = math.radians(self.yaw_deg)
        accel_forward = 0.16 * desired_forward
        accel_right = 0.16 * desired_right
        accel_north = math.cos(yaw) * accel_forward - math.sin(yaw) * accel_right
        accel_east = math.sin(yaw) * accel_forward + math.cos(yaw) * accel_right
        self.vn += (accel_north + self.wind_accel_north - 0.8 * self.vn) * dt
        self.ve += (accel_east + self.wind_accel_east - 0.8 * self.ve) * dt
        self.north_m += self.vn * dt
        self.east_m += self.ve * dt


class SimulationClock:
    def __init__(self, vehicle: SimulationVehicle) -> None:
        self.vehicle = vehicle

    def monotonic(self) -> float:
        return self.vehicle.time

    def sleep(self, seconds: float) -> None:
        self.vehicle.advance(max(0.0, seconds))
