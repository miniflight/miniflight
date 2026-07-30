"""Explicit supervisor facts derived from one observation tick."""

from __future__ import annotations

from dataclasses import dataclass

from miniflight.core.observation import ObservationTick


@dataclass(frozen=True)
class ConditionSet:
    """Facts used by the state machine, never commands or hidden policy."""

    link_live: bool
    imu_fresh: bool
    camera_present: bool
    camera_fresh: bool

    @classmethod
    def from_tick(
        cls,
        tick: ObservationTick,
        *,
        link_live: bool,
        max_imu_age_ns: int,
        max_camera_age_ns: int,
    ) -> "ConditionSet":
        """Derive freshness facts from one tick and declared time limits."""
        if max_imu_age_ns < 0 or max_camera_age_ns < 0:
            raise ValueError("freshness limits must be non-negative")
        camera_age_ns = (
            None
            if tick.camera_timestamp_ns is None
            else tick.timestamp_ns - tick.camera_timestamp_ns
        )
        return cls(
            link_live=link_live,
            imu_fresh=link_live and tick.source_age_ns <= max_imu_age_ns,
            camera_present=tick.has_camera,
            camera_fresh=(
                link_live
                and camera_age_ns is not None
                and camera_age_ns <= max_camera_age_ns
            ),
        )

    @property
    def visual_hold_ready(self) -> bool:
        """Return true only when both IMU and camera are fresh on a live link."""
        return self.link_live and self.imu_fresh and self.camera_fresh
