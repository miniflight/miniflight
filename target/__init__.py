from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from miniflight import Command, State


class Target:
    commands: frozenset[type] = frozenset()

    def connect(self) -> None:
        raise NotImplementedError

    def disconnect(self) -> None:
        raise NotImplementedError

    def read(self, timeout=1.0) -> "State":
        raise NotImplementedError

    def send(self, command: "Command") -> None:
        raise NotImplementedError

    def arm(self) -> None:
        raise NotImplementedError

    def disarm(self) -> None:
        raise NotImplementedError
