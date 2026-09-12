from __future__ import annotations

import math


EARTH_RADIUS_M = 6_371_008.8


def local_ne_m(origin_lat: float, origin_lon: float, lat: float, lon: float) -> tuple[float, float]:
    """Small-area WGS84 approximation: current point in North/East metres."""
    d_lat = math.radians(lat - origin_lat)
    d_lon = math.radians(lon - origin_lon)
    mean_lat = math.radians((lat + origin_lat) * 0.5)
    return EARTH_RADIUS_M * d_lat, EARTH_RADIUS_M * math.cos(mean_lat) * d_lon


def ne_to_body(north: float, east: float, yaw_deg: float) -> tuple[float, float]:
    """Rotate a North/East vector into body forward/right coordinates."""
    yaw = math.radians(yaw_deg)
    forward = math.cos(yaw) * north + math.sin(yaw) * east
    right = -math.sin(yaw) * north + math.cos(yaw) * east
    return forward, right


def circular_mean_deg(values: list[float]) -> float:
    if not values:
        raise ValueError("Cannot average an empty angle sequence")
    s = sum(math.sin(math.radians(value)) for value in values)
    c = sum(math.cos(math.radians(value)) for value in values)
    result = math.degrees(math.atan2(s, c)) % 360.0
    return 0.0 if math.isclose(result, 360.0, abs_tol=1e-12) else result
