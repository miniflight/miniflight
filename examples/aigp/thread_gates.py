import time

from miniflight import Vehicle
from target.aigp import SimulatorClient


GATES = (
    (-23.297967744257804, -0.3999021772411884, -1.3919580206274782),
    (-46.89374907055175, -2.499990058329445, 3.7080417871475424),
    (-74.5937498334912, 1.200009870144981, 12.308041214942953),
    (-111.49374372997558, -5.099989724543434, 23.208040833473227),
    (-135.4937437299756, -0.7999900702503742, 23.99565374851229),
    (-159.19374067821778, -4.399989915278297, 24.6080404520035),
)

RATE_HZ = 20.0
HOLD_SECONDS = 12.0


vehicle = Vehicle(SimulatorClient())
vehicle.connect()

try:
    vehicle.arm()
    time.sleep(1.0)

    for gate in GATES:
        deadline = time.monotonic() + HOLD_SECONDS

        while time.monotonic() < deadline:
            vehicle.position_ned(*gate)
            time.sleep(1.0 / RATE_HZ)
finally:
    try:
        vehicle.disarm()
    finally:
        vehicle.disconnect()
