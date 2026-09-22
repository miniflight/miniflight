import time

from pymavlink import mavutil

from target.aigp import SimulatorClient


ARMED = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED


def wait_armed(target, expected):
    deadline = time.monotonic() + 5.0

    while True:
        remaining = deadline - time.monotonic()

        if remaining <= 0:
            raise TimeoutError(f"VQ1 armed state did not become {expected}")

        heartbeat = target._message("HEARTBEAT", timeout=remaining)

        armed = bool(heartbeat.base_mode & ARMED)
        print(f"base_mode={heartbeat.base_mode} armed={armed}")

        if armed == expected:
            return


target = SimulatorClient(camera_port=None)

print("Connecting to VQ1...")
target.connect()

try:
    wait_armed(target, False)

    print("Requesting arm...")
    target.arm()
    wait_armed(target, True)
    print("Arm confirmed")

    print("Requesting disarm...")
    target.disarm()
    wait_armed(target, False)
    print("Disarm confirmed")
finally:
    try:
        target.disarm()
    finally:
        target.disconnect()
        print("Disconnected from VQ1")
