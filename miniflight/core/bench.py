"""Declarative, props-removed contract for one actuator bench test."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class BenchPlan:
    """One short output test that later code may execute only with interlocks.

    This object contains no transport or motor command path. It is valid only
    for a props-removed bench test. A later executor must require a live link,
    explicit operator enable, and an independent timeout.
    """

    MAX_DURATION_MS = 500

    output_index: int
    command_fraction: float
    duration_ms: int
    expected_observation: str

    def __post_init__(self) -> None:
        if self.output_index < 0:
            raise ValueError("output_index must be non-negative")
        if not math.isfinite(self.command_fraction) or not 0.0 < self.command_fraction <= 1.0:
            raise ValueError("command_fraction must be in (0, 1]")
        if not 0 < self.duration_ms <= self.MAX_DURATION_MS:
            raise ValueError(f"duration_ms must be in [1, {self.MAX_DURATION_MS}]")
        if not self.expected_observation.strip():
            raise ValueError("expected_observation is required")
