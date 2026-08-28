import time

from pymavlink import mavutil

from target import Target


class VQ1(Target):
    def __init__(self):
        self._link = None
        self._boot_ms = None
        self._messages = {}

    def connect(self) -> None:
        self._boot_ms = int(time.time() * 1000)
        self._link = mavutil.mavlink_connection("udpin:127.0.0.1:14550")

        heartbeat = self._link.wait_heartbeat()
        print(heartbeat)

    def disconnect(self) -> None:
        if self._link is not None:
            self._link.close()

        self._link = None
        self._boot_ms = None
        self._messages.clear()

    def arm(self) -> None:
        self._set_armed(True)

    def disarm(self) -> None:
        self._set_armed(False)

    def position(self) -> tuple[float, float, float]:
        while "LOCAL_POSITION_NED" not in self._messages:
            self._receive(blocking=True)

        while self._receive(blocking=False):
            pass

        message = self._messages["LOCAL_POSITION_NED"]
        return message.x, message.y, message.z

    def position_ned(
        self,
        north: float,
        east: float,
        down: float,
    ) -> None:
        mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )
        self._link.mav.set_position_target_local_ned_send(
            int(time.time() * 1000) - self._boot_ms,
            self._link.target_system,
            self._link.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            mask,
            north,
            east,
            down,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )

    def _receive(self, blocking: bool) -> bool:
        message = self._link.recv_match(blocking=blocking)

        if message is None:
            return False

        if message.get_type() != "BAD_DATA":
            self._messages[message.get_type()] = message

        return True

    def _set_armed(self, armed: bool) -> None:
        self._link.mav.command_long_send(
            self._link.target_system,
            self._link.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            int(armed),
            0,
            0,
            0,
            0,
            0,
            0,
        )
