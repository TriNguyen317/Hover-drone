"""Near-level flow velocity aid. Mapping MUST be verified on the actual aircraft."""
import math
import struct

from .msp import MSPError


class FlowWatchdog:
    """Debounce short invalid samples while enforcing a hard loss timeout."""
    def __init__(self, timeout_s):
        self.timeout_s = timeout_s
        self.invalid_since = None

    def update(self, valid, now):
        if valid:
            self.invalid_since = None
            return "ok"
        if self.invalid_since is None:
            self.invalid_since = now
        return "lost" if now - self.invalid_since >= self.timeout_s else "grace"


def decode_flow(payload):
    if len(payload) != 9:
        raise MSPError(f"Invalid optical-flow payload length: {len(payload)}")
    return struct.unpack("<Bhhhh", payload)


def flow_velocity(sample, status, distance, attitude, config, *, check_speed=True):
    quality, fx, fy, bx, by = sample
    row = dict(flow_quality=quality, flow_status=status, flow_x_dps=fx,
               flow_y_dps=fy, flow_body_x_dps=bx, flow_body_y_dps=by,
               flow_range_m=distance)
    reason = "ok"
    if status != 1:
        reason = "sensor_status"
    elif quality < config.min_quality:
        reason = "quality"
    elif not math.isfinite(distance) or not config.min_range_m <= distance <= config.max_range_m:
        reason = "range"
    elif not all(math.isfinite(v) for v in (attitude.roll_deg, attitude.pitch_deg, attitude.yaw_deg)) or max(abs(attitude.roll_deg), abs(attitude.pitch_deg)) > config.max_tilt_deg:
        reason = "tilt"
    rates = {"x": math.radians(fx - bx), "y": math.radians(fy - by)}
    forward = rates[config.forward_axis] * config.forward_sign * distance * config.scale
    right = rates[config.right_axis] * config.right_sign * distance * config.scale
    yaw = math.radians(attitude.yaw_deg)
    north = forward * math.cos(yaw) - right * math.sin(yaw)
    east = forward * math.sin(yaw) + right * math.cos(yaw)
    if reason == "ok" and not math.isfinite(north + east):
        reason = "nonfinite"
    elif (reason == "ok" and check_speed
          and math.hypot(north, east) > config.max_speed_mps):
        reason = "speed"
    row.update(flow_valid=int(reason == "ok"), flow_reason=reason,
               flow_north_mps=north, flow_east_mps=east)
    return north, east, row
