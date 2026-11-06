#!/usr/bin/env python3
"""orient_filter.py – Orientation & rough position from MSP RAW_IMU.

Pure-Python complementary filter (no deps).

Assumptions:
  * Betaflight / INAV default scalers (1 g ≈ 4096 LSB, 1 °/s ≈ 16.4 LSB).
  * Board X->forward, Y->right, Z->down (BF convention).

Outputs tab-separated:
  t  roll pitch yaw  ax ay az  vx vy vz  px py pz
Units:
  angles in degrees, accel m/s², vel m/s, pos m.

Usage:
  python3 orient_filter.py [serial] [baud]
"""
import math
import os
import select
import struct
import sys
import termios
import time
import glob
from enum import Enum, auto

# Lazy-import PyBullet renderer and simulation classes; guarded in case dependencies are missing
try:
    from sim import Vector3D, Quaternion, Body, World, Renderer
except Exception as _e:
    print(f"[orient_filter] WARNING: Could not import renderer dependencies ({_e}). Visualization disabled.")
    Renderer = None
    World = Body = Vector3D = Quaternion = None

CMD_RAW_IMU = 102
ACC_SCALE = 9.80665 / 4096.0      # m/s² per LSB
GYRO_SCALE = math.radians(1.0) / 16.4  # rad/s per LSB  (≈0.001065 rad/s)
ALPHA = 0.98                      # complementary filter coefficient

# -----------------------------------------------------------------------------

def msp_req(cmd: int) -> bytes:
    ck = cmd  # size=0 so checksum = 0 ^ cmd
    return b"$M<" + bytes([0, cmd, ck])

RAW_REQ = msp_req(CMD_RAW_IMU)

# -----------------------------------------------------------------------------

def open_serial(dev: str, baud: int = 115200) -> int:
    fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attr = termios.tcgetattr(fd)
    attr[0] = attr[1] = attr[3] = 0
    attr[2] = termios.CREAD | termios.CLOCAL | termios.CS8
    bconst = getattr(termios, f"B{baud}", None)
    if bconst is None:
        raise ValueError("bad baud")
    if hasattr(termios, "cfsetispeed"):
        termios.cfsetispeed(attr, bconst)
        termios.cfsetospeed(attr, bconst)
    else:
        attr[4] = attr[5] = bconst
    termios.tcsetattr(fd, termios.TCSANOW, attr)
    return fd


def auto_dev():
    for pat in ("/dev/tty.usbmodem*", "/dev/tty.usbserial*", "/dev/tty.SLAB_USBtoUART*", "/dev/ttyUSB*"):
        lst = glob.glob(pat)
        if lst:
            return lst[0]
    raise FileNotFoundError("no serial dev found")

# -----------------------------------------------------------------------------

def parse(buf: bytearray):
    i = 0
    while True:
        s = buf.find(b"$M>", i)
        if s == -1 or s + 5 > len(buf):
            break
        size = buf[s + 3]
        cmd = buf[s + 4]
        end = s + 5 + size
        if end >= len(buf):
            break
        payload = bytes(buf[s + 5:end])
        crc = buf[end]
        calc = size ^ cmd
        for b in payload:
            calc ^= b
        if calc == crc:
            yield cmd, payload
            i = end + 1
        else:
            i = s + 1
    del buf[:i]

# -----------------------------------------------------------------------------

class PreviewPhase(Enum):
    UNAVAILABLE = auto()
    INITIALIZING = auto()
    ACTIVE = auto()
    FAILED = auto()


def main():
    dev = sys.argv[1] if len(sys.argv) > 1 else auto_dev()
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
    fd = open_serial(dev, baud)
    poll = select.poll(); poll.register(fd, select.POLLIN)
    buf = bytearray()

    # state
    roll = pitch = yaw = 0.0  # rad
    vx = vy = vz = 0.0        # m/s
    px = py = pz = 0.0        # m
    last_t = time.time()

    # ------------------------------------------------------------------
    # Initialize visualization if Renderer is available
    preview = PreviewPhase.UNAVAILABLE if Renderer is None else PreviewPhase.INITIALIZING
    if preview is PreviewPhase.INITIALIZING:
        # Build a minimal world with a single quad body for rendering
        world = World(gravity=None, dt=0.01)
        quad = Body()
        # Tag URDF so Renderer loads the proper model
        quad.urdf_filename = "quadrotor.urdf"
        world.add_body(quad)

        # Instantiate renderer (PyBullet GUI by default)
        try:
            renderer = Renderer(world, config='X', gui=True)
            preview = PreviewPhase.ACTIVE
        except Exception as _e:
            print(f"[orient_filter] WARNING: Failed to initialize renderer ({_e}). Disabling visualization.")
            preview = PreviewPhase.FAILED

    print("t\troll\tpitch\tyaw\tax\tay\taz\tvx\tvy\tvz\tpx\tpy\tpz")
    next_req = 0.0
    try:
        while True:
            now = time.time()
            if now >= next_req:
                os.write(fd, RAW_REQ)
                next_req = now + 0.01  # 100 Hz
            if poll.poll(5):
                try:
                    chunk = os.read(fd, 512)
                    buf.extend(chunk)
                except BlockingIOError:
                    pass
            for cmd, payload in list(parse(buf)):
                if cmd != CMD_RAW_IMU or len(payload) < 12:
                    continue
                ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw = struct.unpack_from("<hhhhhh", payload)
                dt = now - last_t
                last_t = now

                # convert
                ax = ax_raw * ACC_SCALE
                ay = ay_raw * ACC_SCALE
                az = az_raw * ACC_SCALE
                gx = gx_raw * GYRO_SCALE
                gy = gy_raw * GYRO_SCALE
                gz = gz_raw * GYRO_SCALE

                # integrate gyro to angles
                roll += gx * dt
                pitch += gy * dt
                yaw += gz * dt

                # accel tilt angles
                if abs(az) > 1e-3:
                    roll_acc = math.atan2(ay, az)
                    pitch_acc = math.atan2(-ax, math.sqrt(ay * ay + az * az))
                    roll = ALPHA * roll + (1 - ALPHA) * roll_acc
                    pitch = ALPHA * pitch + (1 - ALPHA) * pitch_acc
                # yaw left to gyro (could use mag)

                # rotate body-frame accel to world frame
                cr = math.cos(roll); sr = math.sin(roll)
                cp = math.cos(pitch); sp = math.sin(pitch)
                cy = math.cos(yaw); sy = math.sin(yaw)
                # body->world using ZYX
                ax_w = cp * (cy * ax + sy * ay) + sp * az
                ay_w = sr * sp * (cy * ax + sy * ay) + cr * ay - sr * cp * az
                az_w = -cr * sp * (cy * ax + sy * ay) + sr * ay + cr * cp * az
                # remove gravity (gravity vector ~ +9.81 in sensor Z down) -> world Z up negative?
                az_w -= 9.80665  # basic compensation

                # integrate to velocity & position
                vx += ax_w * dt
                vy += ay_w * dt
                vz += az_w * dt
                px += vx * dt
                py += vy * dt
                pz += vz * dt

                # ------------------------------------------------------
                # Update visualization
                if preview is PreviewPhase.ACTIVE:
                    # Update quad state
                    quad.position = Vector3D(0.0, 0.0, 1.1)
                    quad.velocity = Vector3D(0.0, 0.0, 0.0)
                    quad.orientation = Quaternion.from_euler(roll, pitch, yaw)
                    # Advance renderer (draw updates PyBullet objects)
                    renderer.draw()
                    # Step PyBullet simulation for smooth visuals
                    if hasattr(renderer, 'p'):
                        try:
                            renderer.p.stepSimulation()
                        except Exception:
                            pass

                print(f"{now:.2f}\t{math.degrees(roll):.2f}\t{math.degrees(pitch):.2f}\t{math.degrees(yaw):.2f}\t"
                      f"{ax:.2f}\t{ay:.2f}\t{az:.2f}\t"
                      f"{vx:.2f}\t{vy:.2f}\t{vz:.2f}\t"
                      f"{px:.2f}\t{py:.2f}\t{pz:.2f}")
    except KeyboardInterrupt:
        pass
    finally:
        os.close(fd)


if __name__ == "__main__":
    main()
