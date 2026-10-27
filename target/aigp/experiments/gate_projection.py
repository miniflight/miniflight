"""Project privileged VQ1 gate geometry onto recorded camera frames."""

import json
import math

import numpy as np

from common.math import Quaternion
from miniflight import Frame
from target.aigp.experiments.recording import ReplayClient


def observations(path):
    client = ReplayClient()
    with path.open() as source:
        for row in map(json.loads, source):
            if row["event"] == "call" and row["name"] == "poll":
                packets = client.feed(row.get("result", ()))
                frames = tuple(packet.decoded for packet in packets if isinstance(packet.decoded, Frame))
                yield client.read(timeout=0), frames, client.gates


def project(gate, motion, attitude, width, height, pivot_to_center):
    """Gate local X is width, Z is down; camera X is right, Y down, Z forward."""
    corners = np.array([(-width/2, 0, -height/2), (width/2, 0, -height/2),
                        (width/2, 0, height/2), (-width/2, 0, height/2)]) + pivot_to_center
    world = corners @ Quaternion(*gate.orientation).as_rotation_matrix().T + gate.position
    rotation = Quaternion.from_euler(attitude.roll, attitude.pitch, attitude.yaw).as_rotation_matrix()
    mount = Quaternion.from_euler(0, math.radians(20), 0).as_rotation_matrix()
    camera = ((world - motion.position) @ rotation @ mount)[:, (1, 2, 0)]
    return None if np.any(camera[:, 2] <= .05) else 320 * camera[:, :2] / camera[:, 2:] + (320, 180)
