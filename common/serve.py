"""Shared utilities for the renderer HTTP server used by sim and config."""

from __future__ import annotations

import copy
import json
import os
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional
from urllib.parse import urlparse


class SharedState:
    """Thread-safe container for simulation snapshots and input state."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._changed = threading.Condition(self._lock)
        self._version = 0
        self._snapshot: Dict[str, Any] = {
            "world_time": 0.0,
            "frame": 0,
            "bodies": [],
            "updated_at": time.time(),
        }
        self._input: Dict[str, bool] = {}
        self._reset_yaw = False

    def set_snapshot(self, snapshot: Dict[str, Any]) -> None:
        with self._changed:
            snapshot = copy.deepcopy(snapshot)
            snapshot["updated_at"] = time.time()
            self._snapshot = snapshot
            self._version += 1
            self._changed.notify_all()

    def get_snapshot(self) -> Dict[str, Any]:
        with self._lock:
            return copy.deepcopy(self._snapshot)

    def wait_for_snapshot(
        self, after_version: int, timeout: float = 15.0
    ) -> tuple[int, Dict[str, Any]]:
        """Wait for a new snapshot, then return its sequence and value."""
        with self._changed:
            if self._version <= after_version:
                self._changed.wait(timeout)
            return self._version, copy.deepcopy(self._snapshot)

    def set_input_keys(self, keys: Iterable[str]) -> None:
        normalized = {str(k): True for k in keys}
        with self._lock:
            self._input = normalized

    def get_input_state(self) -> Dict[str, bool]:
        with self._lock:
            return dict(self._input)

    def request_yaw_reset(self) -> None:
        with self._lock:
            self._reset_yaw = True

    def consume_yaw_reset(self) -> bool:
        with self._lock:
            if self._reset_yaw:
                self._reset_yaw = False
                return True
            return False


def _infer_mime_type(path: Path) -> str:
    if path.suffix == ".js":
        return "application/javascript"
    if path.suffix in {".css", ".wasm"}:
        return "text/css" if path.suffix == ".css" else "application/wasm"
    if path.suffix in {".obj", ".mtl"}:
        return "text/plain"
    if path.suffix == ".glb":
        return "model/gltf-binary"
    if path.suffix == ".gltf":
        return "model/gltf+json"
    if path.suffix in {".json"}:
        return "application/json"
    return "text/html"


def _make_handler(shared_state: SharedState, static_dir: Path):
    class RendererRequestHandler(BaseHTTPRequestHandler):
        server_version = "MiniFlightRenderer/1.0"

        def do_GET(self) -> None:  # noqa: N802 (method name from BaseHTTPRequestHandler)
            parsed = urlparse(self.path)
            route = parsed.path

            if route == "/state":
                snapshot = shared_state.get_snapshot()
                payload = json.dumps(snapshot).encode("utf-8")
                self._send_response(200, "application/json", payload)
                return

            if route == "/events":
                self._send_events()
                return

            if route in {"/", "/index.html"}:
                target = static_dir / "index.html"
                self._send_static(target)
                return

            if route.startswith("/js/"):
                target = (static_dir / route.lstrip("/")).resolve()
                if not str(target).startswith(str(static_dir.resolve())):
                    self.send_error(403)
                    return
                self._send_static(target)
                return

            if route.startswith("/data/"):
                target = (static_dir / route.lstrip("/")).resolve()
                if not str(target).startswith(str(static_dir.resolve())):
                    self.send_error(403)
                    return
                self._send_static(target)
                return

            self.send_error(404)

        def do_POST(self) -> None:  # noqa: N802
            parsed = urlparse(self.path)

            if parsed.path == "/reset_yaw":
                shared_state.request_yaw_reset()
                self._send_response(204, "application/json", b"")
                return

            if parsed.path != "/input":
                self.send_error(404)
                return

            content_length = int(self.headers.get("Content-Length", "0"))
            raw = self.rfile.read(content_length) if content_length else b"{}"
            try:
                data = json.loads(raw.decode("utf-8"))
            except json.JSONDecodeError:
                self.send_error(400, "invalid json")
                return

            keys = data.get("keys", [])
            if not isinstance(keys, list):
                self.send_error(400, "keys must be a list")
                return

            shared_state.set_input_keys(keys)
            self._send_response(204, "application/json", b"")

        def _send_static(self, path: Path) -> None:
            try:
                data = path.read_bytes()
            except FileNotFoundError:
                self.send_error(404)
                return
            mime_type = _infer_mime_type(path)
            self._send_response(200, mime_type, data)

        def _send_events(self) -> None:
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "keep-alive")
            self.end_headers()
            version = -1
            try:
                while True:
                    version, snapshot = shared_state.wait_for_snapshot(version)
                    payload = json.dumps(snapshot, separators=(",", ":"))
                    self.wfile.write(
                        f"id: {version}\nevent: state\ndata: {payload}\n\n".encode("utf-8")
                    )
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                return

        def _send_response(self, status: int, content_type: str, payload: bytes) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
            self.end_headers()
            if payload:
                self.wfile.write(payload)

        def log_message(self, fmt: str, *args: Any) -> None:  # noqa: A003 (fmt name)
            # Silence default logging to keep simulator output clean.
            return

    return RendererRequestHandler


class RendererServer:
    """Utility wrapper for running the renderer HTTP server in a thread."""

    def __init__(self, shared_state: SharedState, host: str, port: int, static_dir: Path) -> None:
        handler_cls = _make_handler(shared_state, static_dir)
        self._httpd = ThreadingHTTPServer((host, port), handler_cls)
        self._httpd.daemon_threads = True
        self._thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def shutdown(self) -> None:
        self._httpd.shutdown()
        self._httpd.server_close()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)


def _launch_browser(url: str, delay: float) -> None:
    time.sleep(delay)
    try:
        webbrowser.open(url, new=1, autoraise=True)
    except Exception:
        pass


def should_open_browser(
    open_browser: Optional[bool], *, env_var: str = "MINIFLIGHT_RENDER_OPEN_BROWSER", default: bool = True
) -> bool:
    if open_browser is not None:
        return open_browser

    env_value = os.getenv(env_var)
    if env_value is None:
        return default

    return env_value.strip().lower() not in {"0", "false", "no"}


def maybe_launch_browser(
    url: str,
    *,
    open_browser: Optional[bool] = None,
    env_var: str = "MINIFLIGHT_RENDER_OPEN_BROWSER",
    default: bool = True,
    delay: float = 0.75,
) -> None:
    """Launch the system browser if configuration permits it."""

    if should_open_browser(open_browser, env_var=env_var, default=default):
        threading.Thread(target=_launch_browser, args=(url, delay), daemon=True).start()


__all__ = [
    "SharedState",
    "RendererServer",
    "maybe_launch_browser",
    "should_open_browser",
]
