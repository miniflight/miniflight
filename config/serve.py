"""Serve the Miniflight bench view from one Python USB connection."""
import argparse
from contextlib import contextmanager
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import struct
import threading
import time
from urllib.parse import urlsplit

from target.betaflight import blackbox, msp, telemetry

HERE = Path(__file__).resolve().parent


class Bench:
    def __init__(self, device):
        self.device = device
        self.stop = threading.Event()
        self.usb = threading.Lock()
        self.saving = threading.Lock()
        self.link = None
        self.publish(dict(connection=dict(state="waiting_usb")))

    def publish(self, state):
        self.state = state
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
                        try:
                            self.monitor(link, device)
                        finally:
                            with self.usb:
                                self.link = None
                except (OSError, ValueError, struct.error) as error:
                    self.publish(dict(connection=dict(state="retrying", port=device, message=str(error))))
            self.stop.wait(.5)

    def monitor(self, link, device):
        controller, boxes = telemetry.identity(link), link.read(msp.BOX_IDS)
        update_at = started = time.monotonic()
        samples = frames = 0
        while not self.stop.is_set():
            with self.usb:
                if time.monotonic() >= update_at:
                    health = telemetry.health(link, boxes)
                    outputs = telemetry.outputs(link)
                    flash = blackbox.summary(link)
                    update_at = time.monotonic() + 1
                    frames += 5
                signals = {**outputs, **telemetry.motion(link)}
                samples += 1
                frames += 2
                elapsed = max(time.monotonic()-started, .001)
                hz = samples/elapsed
                self.publish(dict(connection=dict(state="live", port=device, last_frame_at_ms=time.time_ns()//1_000_000),
                                  controller=controller, health=health, signals=signals, blackbox=flash,
                                  machine=dict(has_imu={"ACC", "GYRO"}.issubset(health["sensors"])),
                                  traffic=dict(attitude_samples_total=samples, imu_samples_total=samples,
                                               attitude_per_second=hz, imu_per_second=hz, frames_per_second=frames/elapsed)))
                self.link = link
            self.stop.wait(.015)

    @contextmanager
    def blackbox_export(self):
        if not self.saving.acquire(blocking=False):
            raise OSError("A Blackbox export is already running")
        try:
            with self.usb:
                if self.link is None:
                    raise OSError("Connect a flight controller first")
                flash = blackbox.summary(self.link)
                if not flash["ready"] or not flash["used"]:
                    raise OSError("No Blackbox data is ready to save")
                state = self.state
                self.publish({**state, "blackbox": {**flash, "copied": 0}})
                def chunks():
                    copied = 0
                    for i, data in enumerate(blackbox.blocks(self.link, flash["used"]), 1):
                        if self.stop.is_set():
                            raise InterruptedError("Configurator stopped")
                        copied += len(data)
                        if i % 16 == 0:
                            self.publish({**state, "blackbox": {**flash, "copied": copied}})
                        yield data
                try:
                    yield flash["used"], chunks()
                finally:
                    self.publish({**state, "blackbox": flash})
        finally:
            self.saving.release()


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, bench, **kwargs):
        self.bench = bench
        super().__init__(*args, directory=HERE, **kwargs)

    def do_POST(self):
        if self.path != "/blackbox":
            self.send_error(404)
            return
        if self.headers.get("Origin") != f"http://127.0.0.1:{self.server.server_port}":
            self.send_error(403)
            return
        started = False
        try:
            with self.bench.blackbox_export() as (size, chunks):
                self.connection.settimeout(5)
                self.send_response(200)
                self.send_header("Content-Type", "application/octet-stream")
                self.send_header("Content-Disposition", 'attachment; filename="miniflight-blackbox.bbl"')
                self.send_header("Content-Length", str(size))
                self.end_headers()
                started = True
                for chunk in chunks:
                    self.wfile.write(chunk)
        except (OSError, ValueError, struct.error):
            if started:
                self.close_connection = True
            else:
                self.send_error(503, "Blackbox export failed")

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
