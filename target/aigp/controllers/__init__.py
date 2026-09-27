from miniflight import Command, State


class BaseController:
    def update(self, state: State, gate_index: int) -> Command | None:
        """Return one vehicle command, or None while waiting for required observations.

        state: Current vehicle observations. Unavailable or stale optional samples are None.
        gate_index: The zero-based active gate reported by the simulator.
        """
        raise NotImplementedError
