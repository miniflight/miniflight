import time

from miniflight import Vehicle
from target.aigp import SimulatorClient


RATE_HZ = 250.0
MOVE_SECONDS = 2.0
STOP_SECONDS = 1.0


vehicle = Vehicle(SimulatorClient())
vehicle.connect()

try:
    vehicle.arm()
    time.sleep(1.0)

    vehicle.read()
    print("start", vehicle.position)

    start = time.monotonic()
    deadline = start + MOVE_SECONDS
    commands = 0

    while time.monotonic() < deadline:
        vehicle.velocity_ned(-1.0, 0.0, 0.0)
        commands += 1
        time.sleep(1.0 / RATE_HZ)

    elapsed = time.monotonic() - start
    print("command rate", commands / elapsed)
    vehicle.read()
    print("after move", vehicle.position)

    deadline = time.monotonic() + STOP_SECONDS

    while time.monotonic() < deadline:
        vehicle.velocity_ned(0.0, 0.0, 0.0)
        time.sleep(1.0 / RATE_HZ)

    vehicle.read()
    print("after stop", vehicle.position)
    print("velocity", vehicle.velocity)
finally:
    try:
        vehicle.velocity_ned(0.0, 0.0, 0.0)
        vehicle.disarm()
    finally:
        vehicle.disconnect()
