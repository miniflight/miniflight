from target.aigp.client import SimulatorClient


client = SimulatorClient(camera_port=None)
try:
    client.connect()
    while True:
        for packet in client.poll(.1):
            if packet.data.get_type() == "LOCAL_POSITION_NED":
                print(packet.data)
                raise SystemExit
finally:
    client.disconnect()
