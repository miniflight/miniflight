import time

from pymavlink import mavutil

from target import Target


class VQ1(Target):
    def __init__(self):
        self._link = None
        self._boot_ms = None

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

    def arm(self) -> None:
        self._set_armed(True)

    def disarm(self) -> None:
        self._set_armed(False)

    def position(self) -> tuple[float, float, float]:
        message = self._message("LOCAL_POSITION_NED")
        return message.x, message.y, message.z

    def velocity(self) -> tuple[float, float, float]:
        message = self._message("LOCAL_POSITION_NED")
        return message.vx, message.vy, message.vz

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

    def _message(self, message_type: str):
        message = self._link.recv_match(
            type=message_type,
            blocking=True,
        )

        return message

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
