from __future__ import annotations

import math
from dataclasses import dataclass

from .config import PIDConfig


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass(frozen=True)
class PIDTerms:
    output: float
    p: float
    i: float
    d: float
    error: float


class PID:
    """PID with derivative-on-measurement, filtering and conditional integration."""

    def __init__(self, config: PIDConfig) -> None:
        self.config = config
        self.integral = 0.0
        self._last_measurement: float | None = None
        self._filtered_derivative = 0.0

    def reset(self, measurement: float | None = None) -> None:
        self.integral = 0.0
        self._last_measurement = measurement
        self._filtered_derivative = 0.0

    def update(
        self,
        setpoint: float,
        measurement: float,
        dt: float,
        *,
        integrate: bool = True,
    ) -> PIDTerms:
        if not math.isfinite(setpoint) or not math.isfinite(measurement):
            raise ValueError("PID inputs must be finite")
        if dt <= 0:
            raise ValueError("PID dt must be positive")

        error = setpoint - measurement
        p = self.config.kp * error
        derivative = 0.0
        if self._last_measurement is not None:
            raw = -(measurement - self._last_measurement) / dt
            if self.config.derivative_filter_hz > 0:
                tau = 1.0 / (2.0 * math.pi * self.config.derivative_filter_hz)
                alpha = dt / (tau + dt)
                self._filtered_derivative += alpha * (raw - self._filtered_derivative)
                derivative = self._filtered_derivative
            else:
                derivative = raw
        self._last_measurement = measurement
        d = self.config.kd * derivative

        candidate_integral = self.integral
        if integrate:
            candidate_integral = clamp(
                self.integral + error * dt,
                -self.config.integrator_limit,
                self.config.integrator_limit,
            )
        candidate = p + self.config.ki * candidate_integral + d
        limit = self.config.output_limit
        saturated_high = candidate > limit and error > 0
        saturated_low = candidate < -limit and error < 0
        if integrate and not (saturated_high or saturated_low):
            self.integral = candidate_integral

        i = self.config.ki * self.integral
        output = clamp(p + i + d, -limit, limit)
        return PIDTerms(output=output, p=p, i=i, d=d, error=error)
