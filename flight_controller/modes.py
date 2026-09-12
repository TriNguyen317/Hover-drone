from __future__ import annotations

import statistics
from collections import defaultdict
from typing import Iterable, Sequence

from .models import ModeRange


class SafetyStop(RuntimeError):
    pass


class ManualTakeover(SafetyStop):
    pass


def mode_key(name: str) -> str:
    return " ".join(name.upper().split())


def ranges_by_name(ranges: Iterable[ModeRange]) -> dict[str, list[ModeRange]]:
    result: dict[str, list[ModeRange]] = defaultdict(list)
    for item in ranges:
        result[mode_key(item.name)].append(item)
    return dict(result)


def validate_mode_configuration(
    ranges: Sequence[ModeRange], takeover_index: int, takeover_on_pwm: int
) -> None:
    by_name = ranges_by_name(ranges)
    for name in ("ARM", "ANGLE", "MSP RC OVERRIDE"):
        found = by_name.get(name, [])
        if len(found) != 1:
            raise SafetyStop(f"Mode '{name}' must have exactly one configured AUX range")
    override = by_name["MSP RC OVERRIDE"][0]
    if override.channel_index != takeover_index or not override.contains(takeover_on_pwm):
        raise SafetyStop("MSP RC OVERRIDE does not match the configured physical takeover channel")
    for name in ("ARM", "ANGLE"):
        if by_name[name][0].channel_index == takeover_index:
            raise SafetyStop(f"{name} must not share the physical takeover channel")


def choose_channels(
    ranges: Sequence[ModeRange],
    enabled_modes: set[str],
    channel_count: int,
    takeover_index: int,
    *,
    roll: int = 1500,
    pitch: int = 1500,
    throttle: int = 1000,
    yaw: int = 1500,
) -> list[int]:
    enabled = {mode_key(name) for name in enabled_modes}
    values = [1500] * channel_count
    values[:4] = [int(roll), int(pitch), int(yaw), int(throttle)]  # internal A,E,R,T

    by_channel: dict[int, list[ModeRange]] = defaultdict(list)
    for item in ranges:
        by_channel[item.channel_index].append(item)

    candidates = list(range(925, 2076, 25))
    off_preference = (1000, 1500, 2000, 1250, 1750)
    for channel in range(4, channel_count):
        channel_ranges = by_channel.get(channel, [])
        if channel == takeover_index:
            values[channel] = 1000
            continue
        wanted = [item for item in channel_ranges if mode_key(item.name) in enabled]
        if wanted:
            valid = [
                pwm
                for pwm in candidates
                if all(item.contains(pwm) for item in wanted)
                and all(not item.contains(pwm) for item in channel_ranges if item not in wanted)
            ]
            if not valid:
                raise SafetyStop(f"No safe PWM enables requested mode(s) on CH{channel + 1}")
            centers = [(item.start_pwm + item.end_pwm - 25) / 2 for item in wanted]
            values[channel] = min(valid, key=lambda pwm: abs(pwm - statistics.mean(centers)))
        else:
            valid = [pwm for pwm in off_preference if all(not item.contains(pwm) for item in channel_ranges)]
            if not valid:
                raise SafetyStop(f"No safe OFF PWM on CH{channel + 1}")
            values[channel] = valid[0]
    return values
