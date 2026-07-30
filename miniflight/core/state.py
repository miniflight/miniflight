"""Explicit lifecycle for one Miniflight flight session."""

from __future__ import annotations


class FlightStateMachine:
    """Allow only safe, visible transitions through a flight session.

    This class has no board, sensor, program, motor, or clock dependency.
    It only guards the lifecycle that later classes must obey.
    """

    OFFLINE = "offline"
    OBSERVING = "observing"
    QUALIFYING = "qualifying"
    CALIBRATING = "calibrating"
    READY = "ready"
    RUNNING = "running"
    FAILSAFE = "failsafe"

    _NEXT = {
        OFFLINE: frozenset({OBSERVING}),
        OBSERVING: frozenset({OFFLINE, QUALIFYING}),
        QUALIFYING: frozenset({OBSERVING, CALIBRATING, FAILSAFE}),
        CALIBRATING: frozenset({OBSERVING, READY, FAILSAFE}),
        READY: frozenset({OFFLINE, OBSERVING, RUNNING, FAILSAFE}),
        RUNNING: frozenset({READY, FAILSAFE}),
        FAILSAFE: frozenset({OFFLINE, OBSERVING}),
    }

    def __init__(self) -> None:
        self._state = self.OFFLINE
        self._reason = "new session"

    @property
    def state(self) -> str:
        """Return the current lifecycle state."""
        return self._state

    @property
    def reason(self) -> str:
        """Return the reason for the current lifecycle state."""
        return self._reason

    def transition(self, target: str, reason: str) -> None:
        """Move to one allowed state, or reject the request without mutation."""
        if target not in self._NEXT:
            raise ValueError(f"unknown flight state: {target}")
        if target not in self._NEXT[self._state]:
            raise RuntimeError(f"cannot transition from {self._state} to {target}")
        if not reason:
            raise ValueError("transition reason is required")
        self._state = target
        self._reason = reason
