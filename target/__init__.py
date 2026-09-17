class Target:
    def connect(self) -> None:
        raise NotImplementedError

    def disconnect(self) -> None:
        raise NotImplementedError

    def position(self) -> tuple[float, float, float]:
        raise NotImplementedError

    def velocity(self) -> tuple[float, float, float]:
        raise NotImplementedError

    def arm(self) -> None:
        raise NotImplementedError

    def disarm(self) -> None:
        raise NotImplementedError

    def position_ned(
        self,
        north: float,
        east: float,
        down: float,
    ) -> None:
        raise NotImplementedError

    def velocity_ned(
        self,
        north: float,
        east: float,
        down: float,
    ) -> None:
        raise NotImplementedError

    def body_rates(
        self,
        roll: float,
        pitch: float,
        yaw: float,
        thrust: float,
    ) -> None:
        raise NotImplementedError
