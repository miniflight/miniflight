"""One continuous camera recording through both gates, with logged tracking."""

import base64
from bisect import bisect_right
import json
from pathlib import Path
import subprocess
import sys

import cv2
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from miniflight.state import Frame
from target.aigp.client import _Camera
from . import vision


GREEN, AMBER, BLUE = (90, 245, 110), (30, 195, 255), (255, 190, 70)


def dashed(image, corners, color):
    points = np.asarray(corners)
    for a, b in zip(points, np.roll(points, -1, axis=0), strict=True):
        length = np.linalg.norm(b-a)
        if length > 100000:
            continue
        steps = max(1, int(length/7))
        for i in range(0, steps, 2):
            p = a+(b-a)*(i/steps)
            q = a+(b-a)*(min(i+1, steps)/steps)
            cv2.line(image, tuple(np.round(p).astype(int)), tuple(np.round(q).astype(int)), color, 1, cv2.LINE_AA)


def draw_measured(image, corners, color):
    points = np.round(corners).astype(np.int32)
    cv2.polylines(image, [points], True, color, 1, cv2.LINE_AA)
    for point in points:
        if 0 <= point[0] < 640 and 0 <= point[1] < 360:
            cv2.circle(image, tuple(point), 3, color, -1, cv2.LINE_AA)


def render(root):
    path = root/"flight.jsonl"
    updates = [r for r in map(json.loads, path.open()) if r["event"] == "update" and r.get("signals", {}).get("phase") in ("TRACKING", "FINAL_IMU", "HANDOFF")]
    if not updates:
        raise ValueError("Recording has no approach frames")
    accepted, ahead = {}, {}
    clock = {}
    for row in updates:
        state = row["signals"]
        if state.get("image_id") is not None and "image_corners" in state:
            accepted.setdefault((state["gate_number"], state["image_id"]), row)
        candidate = state.get("lookahead")
        if candidate is not None:
            ahead.setdefault(candidate["image_id"], candidate)
    for row in map(json.loads, path.open()):
        if row["event"] == "call" and row["name"] == "camera_clock":
            clock.update({(frame, stamp):(low+high)/2 for frame, stamp, low, high in row.get("result", [])})
    hosts = [row["host_time"] for row in updates]
    states = []
    for row in updates:
        state = row["signals"]
        if not states or state["imu_time"] > states[-1]["imu_time"]:
            states.append(state)
    times = np.array([s["imu_time"] for s in states])
    positions = np.array([s["position"] for s in states])
    if len(states) < 2:
        raise ValueError("Recording has fewer than two approach poses")
    rotation = Slerp(times, Rotation.from_matrix([s["camera_orientation"] for s in states]))
    score = json.loads((root/"score.json").read_text())
    start = times[0]
    crossing = score["gates"][1]["crossing_imu_time_s"]
    end = times[-1] if crossing is None else min(times[-1], crossing+.25)
    camera = _Camera()
    output = root/"two-gates-camera-imu.mp4"
    command = ["ffmpeg", "-v", "error", "-n", "-f", "rawvideo", "-pixel_format", "bgr24", "-video_size", "640x408",
        "-framerate", "30", "-i", "pipe:0", "-vf", "scale=1280:816", "-an", "-c:v", "libx264", "-preset", "fast",
        "-crf", "18", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(output)]
    previous = None
    last_tick = 0
    frames = []
    with subprocess.Popen(command, stdin=subprocess.PIPE) as writer:
        for row in map(json.loads, path.open()):
            if row["event"] != "call" or row["name"] != "poll":
                continue
            for packet in row.get("result", []):
                if not packet["camera"]:
                    continue
                frame = camera.receive(base64.b64decode(packet["wire"]), packet["received_at"])
                if not isinstance(frame, Frame):
                    continue
                t = clock.get((frame.id, frame.time_ns))
                if t is None or not start <= t <= end:
                    continue
                current = updates[max(0, bisect_right(hosts, frame.received_at)-1)]
                state = current["signals"]
                gate = state["gate_number"]
                image = frame.bgr.copy()
                measured = accepted.get((gate, frame.id))
                mode, color = "IMU prediction", AMBER
                detail = "handoff using retained camera observation"
                if measured is not None:
                    observed = measured["signals"]
                    corners = np.array(observed["image_corners"])
                    draw_measured(image, corners, GREEN)
                    mode, color = "camera tracking", GREEN
                    detail = "accepted aperture corners"
                    if np.ptp(corners, axis=0).max() < 90:
                        center = corners.mean(axis=0)
                        side = int(max(55, np.ptp(corners, axis=0).max()*2.2))
                        x = int(np.clip(center[0]-side/2, 0, 640-side))
                        y = int(np.clip(center[1]-side/2, 0, 360-side))
                        crop = cv2.resize(image[y:y+side, x:x+side], (156, 156), interpolation=cv2.INTER_NEAREST)
                        image[22:178, 474:630] = crop
                        cv2.rectangle(image, (473, 21), (630, 178), GREEN, 1)
                        cv2.putText(image, "aperture crop", (477, 14), 0, .38, GREEN, 1, cv2.LINE_AA)
                elif "gate_corners" in state:
                    p = np.array([np.interp(t, times, axis) for axis in positions.T])
                    xyz = (np.array(state["gate_corners"])-p) @ rotation(t).as_matrix()
                    if np.min(xyz[:, 2]) > .05:
                        pixels = xyz[:, :2]/xyz[:, 2:]*320+[320, 180]
                        dashed(image, pixels, AMBER)
                    detail = f'last image {max(0., t-state["image_time"]):.2f} s old'
                if gate == 1 and frame.id in ahead:
                    draw_measured(image, np.array(ahead[frame.id]["image_corners"]), BLUE)
                    detail += " | blue: next candidate"
                if state.get("height_alignment"):
                    detail += " | aligning height"
                panel = cv2.copyMakeBorder(image, 48, 0, 0, 0, cv2.BORDER_CONSTANT, value=(20, 23, 28))
                cv2.putText(panel, f"Gate {gate} | {mode}", (9, 20), 0, .53, color, 1, cv2.LINE_AA)
                cv2.putText(panel, detail, (9, 40), 0, .39, (220, 225, 230), 1, cv2.LINE_AA)
                passed = current["native_race"]["active_gate_index"]
                cv2.putText(panel, f"SCORER {min(passed, 2)}/2", (505, 20), 0, .46, GREEN, 1, cv2.LINE_AA)
                tick = round((t-start)*30)
                if previous is not None:
                    for _ in range(tick-last_tick):
                        writer.stdin.write(previous.tobytes())
                previous, last_tick = panel, tick
                frames.append(dict(id=frame.id, stamp=frame.time_ns, capture_s=t, gate=gate, mode=mode))
        if previous is None:
            raise ValueError("No camera frames cover the approach")
        writer.stdin.write(previous.tobytes())
        writer.stdin.close()
        if writer.wait() != 0:
            raise RuntimeError("two-gate video encoding failed")
    (root/"video.json").write_text(json.dumps(dict(fps=30, frames=frames, video_frames=last_tick+1,
        duration_s=(last_tick+1)/30, source="one continuous recorded camera stream; no cuts between gates"), indent=2)+"\n")
    print(output)


if __name__ == "__main__":
    render(Path(sys.argv[1]))
