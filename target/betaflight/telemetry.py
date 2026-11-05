"""Decode the Betaflight reads used by the bench display."""
import struct

from target.betaflight import msp


def identity(link):
    if link.read(msp.FC_VARIANT) != b"BTFL":
        raise ValueError("The USB device did not identify as Betaflight")
    api, version, board = (link.read(cmd) for cmd in (msp.API_VERSION, msp.FC_VERSION, msp.BOARD_INFO))
    _, major, minor = struct.unpack_from("<BBB", api)
    firmware = ".".join(str(x) for x in struct.unpack_from("<BBB", version))
    name = board[:4].decode("ascii")
    if len(board) > 8:
        name_offset = 9 + board[8]
        if name_offset < len(board):
            length = board[name_offset]
            name = board[name_offset+1:name_offset+1+length].decode("ascii") or name
    return dict(variant="Betaflight", api_version=f"{major}.{minor}", version=firmware, board=name)


def health(link, box_ids):
    cycle, errors, mask, modes, _, load = struct.unpack_from("<HHHIBH", link.read(msp.STATUS))
    sensors = [name for i, name in enumerate(("ACC", "BARO", "MAG", "GPS", "RANGE", "GYRO", "FLOW")) if mask & (1 << i)]
    arm = box_ids.find(b"\0")
    return dict(loop_hz=round(1e6/cycle) if cycle else None, i2c_errors=errors, sensors=sensors,
                cpu_load_percent=load, armed=bool(modes & (1 << arm)) if 0 <= arm < 32 else None)


def motion(link):
    imu = struct.unpack("<9h", link.read(msp.RAW_IMU))
    roll, pitch, yaw = struct.unpack("<3h", link.read(msp.ATTITUDE))
    return dict(attitude_deg=(roll/10, pitch/10, yaw), accel_msp=imu[:3], gyro_msp=imu[3:6])


def outputs(link):
    voltage, mah, rssi, current = struct.unpack_from("<BHHh", link.read(msp.ANALOG))
    motors = list(n for (n,) in struct.iter_unpack("<H", link.read(msp.MOTOR)))
    while motors and not motors[-1]:
        motors.pop()
    altitude, vario = struct.unpack_from("<ih", link.read(msp.ALTITUDE))
    return dict(battery_v=voltage/10, current_a=current/100, mah=mah, rssi_percent=rssi*100/1023,
                motors=motors, altitude_m=altitude/100, vario_mps=vario/100)
