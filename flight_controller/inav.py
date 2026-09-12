from __future__ import annotations

import struct
from typing import Protocol, Sequence, runtime_checkable

from .models import (
    APIVersion,
    AltitudeData,
    AttitudeData,
    GPSData,
    LocalPoseData,
    ModeRange,
    SensorStatus,
)
from .msp import MSPError, MSPTransport


MSP_API_VERSION = 1
MSP_FC_VARIANT = 2
MSP_FC_VERSION = 3
MSP_MODE_RANGES = 34
MSP_SONAR_ALTITUDE = 58
MSP_RX_MAP = 64
MSP_REBOOT = 68
MSP_MOTOR = 104
MSP_RC = 105
MSP_RAW_GPS = 106
MSP_ATTITUDE = 108
MSP_ALTITUDE = 109
MSP_ACTIVEBOXES = 113
MSP_BOXNAMES = 116
MSP_BOXIDS = 119
MSP_SENSOR_STATUS = 151
MSP_SET_RAW_RC = 200
MSP2_INAV_ESC_RPM = 0x2040
MSP2_INAV_FULL_LOCAL_POSE = 0x2220


KNOWN_BOX_NAMES = {
    0: "ARM",
    1: "ANGLE",
    3: "NAV ALTHOLD",
    10: "NAV RTH",
    11: "NAV POSHOLD",
    28: "NAV WP",
    31: "GCS NAV",
    50: "MSP RC OVERRIDE",
    51: "PREARM",
}


@runtime_checkable
class Vehicle(Protocol):
    def optical_flow(self) -> tuple[int, int, int, int, int]: ...
    def local_pose(self) -> LocalPoseData: ...
    def api_version(self) -> APIVersion: ...
    def fc_variant(self) -> str: ...
    def fc_version(self) -> tuple[int, int, int]: ...
    def sensor_status(self) -> SensorStatus: ...
    def gps(self) -> GPSData: ...
    def altitude(self) -> AltitudeData: ...
    def rangefinder_m(self) -> float: ...
    def attitude(self) -> AttitudeData: ...
    def rc_channels(self) -> list[int]: ...
    def rx_map(self) -> list[int]: ...
    def mode_ranges(self) -> list[ModeRange]: ...
    def active_modes(self) -> set[str]: ...
    def motor_outputs(self) -> list[int]: ...
    def motor_rpm(self) -> list[int]: ...
    def set_raw_rc(self, channels: Sequence[int], rx_map: Sequence[int]) -> None: ...


class INAVClient:
    def optical_flow(self):
        from .optical_flow import decode_flow
        return decode_flow(self.transport.request_v2(0x2001))

    def local_pose(self) -> LocalPoseData:
        payload = self.transport.request_v2(MSP2_INAV_FULL_LOCAL_POSE)
        if len(payload) != 24:
            raise MSPError(
                f"Invalid MSP2_INAV_FULL_LOCAL_POSE length {len(payload)} (expected 24)"
            )
        roll, pitch, yaw = struct.unpack_from("<hhh", payload, 0)
        values = []
        offset = 6
        for _ in range(3):
            position_cm, velocity_cms = struct.unpack_from("<ih", payload, offset)
            values.extend((position_cm / 100.0, velocity_cms / 100.0))
            offset += 6
        return LocalPoseData(
            roll / 10.0, pitch / 10.0, yaw / 10.0,
            values[0], values[2], values[4],
            values[1], values[3], values[5],
        )

    def __init__(self, transport: MSPTransport) -> None:
        self.transport = transport
        self._box_catalog: list[tuple[str, int]] | None = None

    def api_version(self) -> APIVersion:
        payload = self.transport.request(MSP_API_VERSION)
        if len(payload) != 3:
            raise MSPError(f"Invalid MSP_API_VERSION length {len(payload)}")
        return APIVersion(*payload)

    def fc_variant(self) -> str:
        return self.transport.request(MSP_FC_VARIANT).decode("ascii", errors="replace")

    def fc_version(self) -> tuple[int, int, int]:
        payload = self.transport.request(MSP_FC_VERSION)
        if len(payload) != 3:
            raise MSPError(f"Invalid MSP_FC_VERSION length {len(payload)}")
        return payload[0], payload[1], payload[2]

    def sensor_status(self) -> SensorStatus:
        payload = self.transport.request(MSP_SENSOR_STATUS)
        if len(payload) < 9:
            raise MSPError(f"Invalid MSP_SENSOR_STATUS length {len(payload)}")
        values = payload[:9]
        return SensorStatus(bool(values[0]), *values[1:9])

    def gps(self) -> GPSData:
        payload = self.transport.request(MSP_RAW_GPS)
        if len(payload) < 18:
            raise MSPError(f"Invalid MSP_RAW_GPS length {len(payload)}")
        fix, satellites = payload[0], payload[1]
        lat, lon = struct.unpack_from("<ii", payload, 2)
        _alt_m, speed_cms, course, hdop = struct.unpack_from("<HHHH", payload, 10)
        return GPSData(fix, satellites, lat / 1e7, lon / 1e7, hdop / 100.0,
                       speed_cms / 100.0, course / 10.0)

    def altitude(self) -> AltitudeData:
        payload = self.transport.request(MSP_ALTITUDE)
        if len(payload) < 10:
            raise MSPError(f"Invalid MSP_ALTITUDE length {len(payload)}")
        estimated_cm, velocity_cms, baro_cm = struct.unpack_from("<ihi", payload, 0)
        return AltitudeData(estimated_cm / 100.0, velocity_cms / 100.0, baro_cm / 100.0)

    def rangefinder_m(self) -> float:
        payload = self.transport.request(MSP_SONAR_ALTITUDE)
        if len(payload) < 4:
            raise MSPError(f"Invalid MSP_SONAR_ALTITUDE length {len(payload)}")
        return struct.unpack_from("<i", payload, 0)[0] / 100.0

    def attitude(self) -> AttitudeData:
        payload = self.transport.request(MSP_ATTITUDE)
        if len(payload) < 6:
            raise MSPError(f"Invalid MSP_ATTITUDE length {len(payload)}")
        roll_d10, pitch_d10, yaw = struct.unpack_from("<hhH", payload, 0)
        return AttitudeData(roll_d10 / 10.0, pitch_d10 / 10.0, float(yaw % 360))

    def rc_channels(self) -> list[int]:
        payload = self.transport.request(MSP_RC)
        if len(payload) % 2:
            raise MSPError(f"Invalid MSP_RC length {len(payload)}")
        return list(struct.unpack("<" + "H" * (len(payload) // 2), payload))

    def rx_map(self) -> list[int]:
        payload = self.transport.request(MSP_RX_MAP)
        if len(payload) < 4:
            raise MSPError(f"Invalid MSP_RX_MAP length {len(payload)}")
        return list(payload)

    def mode_ranges(self) -> list[ModeRange]:
        payload = self.transport.request(MSP_MODE_RANGES)
        if len(payload) % 4:
            raise MSPError(f"Invalid MSP_MODE_RANGES length {len(payload)}")
        result: list[ModeRange] = []
        for index in range(0, len(payload), 4):
            permanent_id, aux, start, end = payload[index : index + 4]
            if start < end:
                result.append(
                    ModeRange(permanent_id, KNOWN_BOX_NAMES.get(permanent_id, f"BOX ID {permanent_id}"), aux, start, end)
                )
        return result

    def _catalog(self) -> list[tuple[str, int]]:
        if self._box_catalog is not None:
            return self._box_catalog
        ids = list(self.transport.request(MSP_BOXIDS))
        try:
            raw_names = self.transport.request(MSP_BOXNAMES)
            names = [item.decode("ascii", errors="replace") for item in raw_names.split(b";") if item]
        except MSPError:
            names = []
        if len(names) != len(ids):
            names = [KNOWN_BOX_NAMES.get(item, f"BOX ID {item}") for item in ids]
        self._box_catalog = list(zip(names, ids))
        return self._box_catalog

    def active_modes(self) -> set[str]:
        payload = self.transport.request(MSP_ACTIVEBOXES)
        result: set[str] = set()
        for index, (name, _permanent_id) in enumerate(self._catalog()):
            if index // 8 < len(payload) and payload[index // 8] & (1 << (index % 8)):
                result.add(name)
        return result

    def reboot(self) -> None:
        """Request a normal INAV reboot (never DFU/bootloader mode)."""
        self.transport.request(MSP_REBOOT)

    def motor_outputs(self) -> list[int]:
        """Return FC-to-ESC output commands; these values are not measured RPM."""
        payload = self.transport.request(MSP_MOTOR)
        if len(payload) < 8 or len(payload) % 2:
            raise MSPError(f"Invalid MSP_MOTOR length {len(payload)}")
        return list(struct.unpack("<" + "H" * (len(payload) // 2), payload))

    def motor_rpm(self) -> list[int]:
        payload = self.transport.request_v2(MSP2_INAV_ESC_RPM)
        if len(payload) % 4:
            raise MSPError(f"Invalid MSP2_INAV_ESC_RPM length {len(payload)}")
        return list(struct.unpack("<" + "I" * (len(payload) // 4), payload))

    def set_raw_rc(self, channels: Sequence[int], rx_map: Sequence[int]) -> None:
        if len(channels) < 4:
            raise ValueError("At least four AERT channels are required")
        raw = [1500] * len(channels)
        for internal_index, value in enumerate(channels):
            pwm = int(value)
            if not 800 <= pwm <= 2200:
                raise ValueError(f"RC channel {internal_index + 1} outside 800..2200")
            raw_index = rx_map[internal_index] if internal_index < 4 else internal_index
            if raw_index >= len(raw):
                raise ValueError("RX map points outside MSP RC frame")
            raw[raw_index] = pwm
        payload = struct.pack("<" + "H" * len(raw), *raw)
        # MSP_SET_RAW_RC is a continuous input stream. Do not stall the outer
        # control loop waiting for an acknowledgement to every RC frame.
        self.transport.send(MSP_SET_RAW_RC, payload)
