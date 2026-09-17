import struct
import time

from pymavlink import mavutil


UDP_ENDPOINT = "udpin:0.0.0.0:14550"
RATE_HZ = 60
IDLE = [0.05, 0.05, 0.05, 0.05]
TEST = [0.10, 0.05, 0.05, 0.05]
RACE_STATUS_FORMAT = "<BQqqIq"


connection = mavutil.mavlink_connection(UDP_ENDPOINT)
heartbeat = connection.wait_heartbeat(timeout=60)

if heartbeat is None:
    raise SystemExit("No VQ1 heartbeat received")

target_system = heartbeat.get_srcSystem()
target_component = 0


def send_motors(values):
    connection.mav.set_actuator_control_target_send(
        int(time.time() * 1e6),
        0,
        target_system,
        target_component,
        values + [0.0] * 4,
    )


def receive_until(deadline, values):
    latest_output = None
    race_started = False

    while time.monotonic() < deadline:
        send_motors(values)

        while True:
            message = connection.recv_match(blocking=False)

            if message is None:
                break

            if message.get_type() == "ACTUATOR_OUTPUT_STATUS":
                latest_output = [float(value) for value in message.actuator[:4]]

            if message.get_type() == "ENCAPSULATED_DATA":
                payload = bytes(message.data)

                if payload[0] == 1:
                    status = struct.unpack_from(RACE_STATUS_FORMAT, payload)
                    race_started = status[2] >= 0

        time.sleep(1 / RATE_HZ)

    return race_started, latest_output


print(
    f"heartbeat system={target_system} "
    f"component={heartbeat.get_srcComponent()}"
)

connection.mav.command_long_send(
    target_system,
    target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1,
    0,
    0,
    0,
    0,
    0,
    0,
)

ack = connection.recv_match(type="COMMAND_ACK", blocking=True, timeout=2)
print(f"arm_ack={ack.to_dict() if ack else None}")

try:
    race_started = False
    baseline = None
    wait_deadline = time.monotonic() + 30

    while time.monotonic() < wait_deadline and not race_started:
        race_started, output = receive_until(time.monotonic() + 0.25, IDLE)

        if output is not None:
            baseline = output

    print(f"race_started={race_started}")
    print(f"baseline={baseline}")

    if not race_started:
        raise RuntimeError("Race did not start within 30 seconds")

    _, commanded = receive_until(time.monotonic() + 0.25, TEST)
    print(f"commanded={commanded}")
finally:
    _, restored = receive_until(time.monotonic() + 0.5, IDLE)
    print(f"restored={restored}")
