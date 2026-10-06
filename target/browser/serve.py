"""Serve static browser files with isolation headers and byte-range support."""
import argparse
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import re


class Handler(SimpleHTTPRequestHandler):
    extensions_map = {**SimpleHTTPRequestHandler.extensions_map,
                      ".mjs": "text/javascript", ".wasm": "application/wasm"}

    def end_headers(self):
        self.send_header("Cross-Origin-Opener-Policy", "same-origin")
        self.send_header("Cross-Origin-Embedder-Policy", "require-corp")
        self.send_header("Cross-Origin-Resource-Policy", "same-origin")
        super().end_headers()

    def send_head(self):
        requested = self.headers.get("Range")
        if not requested:
            self.remaining = None
            return super().send_head()
        path = Path(self.translate_path(self.path))
        if not path.is_file():
            self.send_error(404)
            return None
        size = path.stat().st_size
        match = re.fullmatch(r"bytes=(\d+)-(\d*)", requested)
        start = int(match[1]) if match else size
        end = min(int(match[2]), size - 1) if match and match[2] else size - 1
        if start > end:
            self.send_response(416)
            self.send_header("Content-Range", "bytes */" + str(size))
            self.send_header("Content-Length", "0")
            self.end_headers()
            return None
        source = path.open("rb")
        source.seek(start)
        self.remaining = end - start + 1
        self.send_response(206)
        self.send_header("Content-Type", self.guess_type(str(path)))
        self.send_header("Content-Length", str(self.remaining))
        self.send_header("Content-Range", f"bytes {start}-{end}/{size}")
        self.send_header("Accept-Ranges", "bytes")
        self.end_headers()
        return source

    def copyfile(self, source, outputfile):
        if self.remaining is None:
            return super().copyfile(source, outputfile)
        while self.remaining:
            block = source.read(min(self.remaining, 1024 * 1024))
            if not block:
                break
            outputfile.write(block)
            self.remaining -= len(block)


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--port", type=int, default=8000)
args = parser.parse_args()
server = ThreadingHTTPServer(("127.0.0.1", args.port), partial(Handler, directory=str(Path(__file__).resolve().parent)))
print(f"http://127.0.0.1:{server.server_port}/", flush=True)
try:
    server.serve_forever()
except KeyboardInterrupt:
    pass
