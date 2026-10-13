"""Serve the viewer and relay each new native JPEG without re-encoding."""

import argparse
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import time


class Handler(SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path != "/camera.mjpg":
            return super().do_GET()
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        path, previous = Path(self.directory) / "frame.jpg", None
        try:
            while True:
                if path.exists() and (stamp := path.stat().st_mtime_ns) != previous:
                    jpeg, previous = path.read_bytes(), stamp
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n")
                    self.wfile.flush()
                time.sleep(.01)
        except (BrokenPipeError, ConnectionResetError):
            return


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    with ThreadingHTTPServer(("127.0.0.1", args.port), partial(Handler, directory=str(args.directory))) as server:
        server.serve_forever()
