"""Explicit safety gate for future props-removed bench execution."""

from __future__ import annotations

from miniflight.core.state import FlightStateMachine


class BenchInterlock:
    """Allow a bench plan only when every required condition is explicit."""

    def __init__(self) -> None:
        self.reason = "not checked"

    def allows(
        self,
        *,
        session_state: str,
        link_live: bool,
        operator_enabled: bool,
        props_removed: bool,
    ) -> bool:
        """Return true only for an explicitly enabled, prepared bench state."""
        if session_state != FlightStateMachine.READY:
            self.reason = "session is not ready"
        elif not link_live:
            self.reason = "link is not live"
        elif not operator_enabled:
            self.reason = "operator has not enabled bench output"
        elif not props_removed:
            self.reason = "propellers are not confirmed removed"
        else:
            self.reason = "allowed"
        return self.reason == "allowed"
