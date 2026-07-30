"""Lockstep supervisor session for one Miniflight connection."""

from __future__ import annotations

from typing import Optional

from miniflight.core.conditions import ConditionSet
from miniflight.core.observation import ObservationTick
from miniflight.core.state import FlightStateMachine


class FlightSession:
    """Accept ordered ticks and advance the supervisor at most once per tick."""

    def __init__(self, max_imu_age_ns: int, max_camera_age_ns: int) -> None:
        if max_imu_age_ns < 0 or max_camera_age_ns < 0:
            raise ValueError("freshness limits must be non-negative")
        self.machine = FlightStateMachine()
        self._max_imu_age_ns = max_imu_age_ns
        self._max_camera_age_ns = max_camera_age_ns
        self._last_sequence: Optional[int] = None

    def step(self, tick: ObservationTick, *, link_live: bool) -> ConditionSet:
        """Process one new tick, then return its explicit supervisor facts."""
        if self._last_sequence is not None and tick.sequence <= self._last_sequence:
            raise RuntimeError("tick sequence must increase")
        conditions = ConditionSet.from_tick(
            tick,
            link_live=link_live,
            max_imu_age_ns=self._max_imu_age_ns,
            max_camera_age_ns=self._max_camera_age_ns,
        )
        self._last_sequence = tick.sequence

        state = self.machine.state
        if not conditions.link_live:
            if state == FlightStateMachine.OBSERVING:
                self.machine.transition(FlightStateMachine.OFFLINE, "link lost")
            elif state not in {FlightStateMachine.OFFLINE, FlightStateMachine.FAILSAFE}:
                self.machine.transition(FlightStateMachine.FAILSAFE, "link lost")
        elif state == FlightStateMachine.OFFLINE:
            self.machine.transition(FlightStateMachine.OBSERVING, "link is live")
        elif state == FlightStateMachine.FAILSAFE:
            self.machine.transition(FlightStateMachine.OBSERVING, "link recovered")
        elif state == FlightStateMachine.OBSERVING and conditions.imu_fresh:
            self.machine.transition(FlightStateMachine.QUALIFYING, "IMU is fresh")
        elif state == FlightStateMachine.QUALIFYING and not conditions.imu_fresh:
            self.machine.transition(FlightStateMachine.OBSERVING, "IMU is stale")

        return conditions
