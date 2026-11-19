"""Serve the viewer and relay each new native JPEG without re-encoding."""

import argparse
from contextlib import contextmanager
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import signal
from threading import Thread
from urllib.parse import urlsplit
import time


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, race_path=None, **kwargs):
        self.race_path = race_path
        super().__init__(*args, **kwargs)

    def log_message(self, format, *args):
        if self.race_path is None:
            super().log_message(format, *args)

    def latest_race(self):
        try:
            with self.race_path.open('rb') as source:
                end = source.seek(0, os.SEEK_END)
                source.seek(max(0, end-8192))
                for line in reversed(source.read().split(b'\n')[:-1]):
                    try:
                        row = json.loads(line)
                    except ValueError:
                        continue
                    if 'active_gate_index' in row:
                        return row
        except FileNotFoundError:
            pass
        return None

    def do_POST(self):
        if self.race_path is None or self.path != '/stop':
            return self.send_error(404)
        host, port = self.server.server_address
        if self.headers.get('Origin') not in (f'http://{host}:{port}', f'http://localhost:{port}'):
            return self.send_error(403)
        self.send_response(202)
        self.send_header('Content-Length', '0')
        self.end_headers()
        self.wfile.flush()
        os.kill(os.getpid(), signal.SIGTERM)

    def do_GET(self):
        if self.path == '/session' and self.race_path is not None:
            try:
                state = json.loads((Path(self.directory)/'session.json').read_text())
            except FileNotFoundError:
                state = {'stage': 'Starting', 'history': []}
            if state['stage'] == 'Flying':
                state['live'] = self.latest_race()
            data = json.dumps(state).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)
            return
        camera = urlsplit(self.path).path == "/camera.mjpg"
        if not camera and self.path != "/signals":
            return super().do_GET()
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame" if camera else "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        path = Path(self.directory) / ("frame.jpg" if camera else "latest.json")
        sent = None
        if camera:
            self.connection.settimeout(2.)
        try:
            while True:
                try:
                    revision = path.stat().st_mtime_ns
                    data = path.read_bytes() if revision != sent else None
                except FileNotFoundError:
                    data = None
                if data is not None:
                    payload = b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(data)).encode() + b"\r\n\r\n" + data + b"\r\n" if camera else b"data: " + data.rstrip() + b"\n\n"
                    self.wfile.write(payload)
                    self.wfile.flush()
                    sent = revision
                time.sleep(.01)
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            return


@contextmanager
def serve_session(directory, race_path, port):
    handler = partial(Handler, directory=str(directory), race_path=race_path)
    with ThreadingHTTPServer(('127.0.0.1', port), handler) as server:
        worker = Thread(target=server.serve_forever, daemon=True)
        worker.start()
        try:
            yield server.server_port
        finally:
            server.shutdown()
            worker.join()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    with ThreadingHTTPServer(("127.0.0.1", args.port), partial(Handler, directory=str(args.directory))) as server:
        server.serve_forever()
