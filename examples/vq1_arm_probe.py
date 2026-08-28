import time

from pymavlink import mavutil

from target.vq1 import VQ1


ARMED = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED


def send_armed(link, armed):
    link.mav.command_long_send(
        link.target_system,
        link.target_component,
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


def wait_armed(link, expected):
    deadline = time.monotonic() + 5.0

    while True:
        remaining = deadline - time.monotonic()

        if remaining <= 0:
            raise TimeoutError(f"VQ1 armed state did not become {expected}")

        heartbeat = link.recv_match(
            type="HEARTBEAT",
            blocking=True,
            timeout=remaining,
        )

        if heartbeat is None:
            raise TimeoutError("VQ1 heartbeat not received")

        armed = bool(heartbeat.base_mode & ARMED)
        print(f"base_mode={heartbeat.base_mode} armed={armed}")

        if armed == expected:
            return


target = VQ1()

print("Connecting to VQ1...")
target.connect()
link = target._link

try:
    wait_armed(link, False)

    print("Requesting arm...")
    send_armed(link, True)
    wait_armed(link, True)
    print("Arm confirmed")

    print("Requesting disarm...")
    send_armed(link, False)
    wait_armed(link, False)
    print("Disarm confirmed")
finally:
    try:
        send_armed(link, False)
    finally:
        target.disconnect()
        print("Disconnected from VQ1")
