"""Serve the Miniflight bench view from one Python USB connection."""
import argparse
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import struct
import threading
import time
from urllib.parse import urlsplit

from target.betaflight import msp, telemetry

HERE = Path(__file__).resolve().parent


class Bench:
    def __init__(self, device):
        self.device = device
        self.stop = threading.Event()
        self.publish(dict(connection=dict(state="waiting_usb")))

    def publish(self, state):
        self.event = b"event: state\ndata: " + json.dumps(state).encode() + b"\n\n"

    def run(self):
        while not self.stop.is_set():
            devices = [self.device] if self.device else sorted(Path("/dev").glob("cu.usbmodem*"))
            if len(devices) != 1:
                message = "Connect the flight controller by USB" if not devices else "Choose a USB port with --device"
                self.publish(dict(connection=dict(state="waiting_usb", message=message)))
            else:
                device = str(devices[0])
                try:
                    self.publish(dict(connection=dict(state="handshake", port=device)))
                    with msp.connect(device) as link:
                        self.monitor(link, device)
                except (OSError, ValueError, struct.error) as error:
                    self.publish(dict(connection=dict(state="retrying", port=device, message=str(error))))
            self.stop.wait(.5)

    def monitor(self, link, device):
        controller, boxes = telemetry.identity(link), link.read(msp.BOX_IDS)
        update_at = started = time.monotonic()
        samples = 0
        while not self.stop.is_set():
            if time.monotonic() >= update_at:
                health = telemetry.health(link, boxes)
                outputs = telemetry.outputs(link)
                update_at = time.monotonic() + 1
            signals = {**outputs, **telemetry.motion(link)}
            samples += 1
            hz = samples/max(time.monotonic()-started, .001)
            self.publish(dict(connection=dict(state="live", port=device, last_frame_at_ms=time.time_ns()//1_000_000),
                              controller=controller, health=health, signals=signals,
                              machine=dict(has_imu={"ACC", "GYRO"}.issubset(health["sensors"])),
                              traffic=dict(attitude_samples_total=samples, imu_samples_total=samples,
                                           attitude_per_second=hz, imu_per_second=hz)))
            self.stop.wait(.015)


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, bench, **kwargs):
        self.bench = bench
        super().__init__(*args, directory=HERE, **kwargs)

    def do_GET(self):
        path = urlsplit(self.path).path
        if path == "/events":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            seen = None
            try:
                while not self.bench.stop.is_set():
                    event = self.bench.event
                    if event is not seen:
                        self.wfile.write(event)
                        self.wfile.flush()
                        seen = event
                    self.bench.stop.wait(.025)
            except (BrokenPipeError, ConnectionResetError):
                pass
        elif path == "/" or path.startswith(("/js/", "/data/")):
            super().do_GET()
        else:
            self.send_error(404)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", help="USB serial device path")
    parser.add_argument("--port", type=int, default=8002)
    args = parser.parse_args()
    bench = Bench(args.device)
    with ThreadingHTTPServer(("127.0.0.1", args.port), partial(Handler, bench=bench)) as server:
        worker = threading.Thread(target=bench.run, daemon=True)
        worker.start()
        print(f"Miniflight is at http://127.0.0.1:{server.server_port}/", flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass
        finally:
            bench.stop.set()
            worker.join(timeout=2)


if __name__ == "__main__":
    main()
