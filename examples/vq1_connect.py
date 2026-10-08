import time

from target.aigp.client import SimulatorClient


client = SimulatorClient(camera_port=None)
try:
    client.open()
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        for packet in client.poll(.1):
            if packet.data.get_type() == "LOCAL_POSITION_NED":
                print(packet.data)
                raise SystemExit
    raise TimeoutError("no native position received")
finally:
    client.disconnect()
