"""Native gate openings for post-flight scoring."""

import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from common.math import Quaternion
from target.aigp.client import _Track
from target.aigp.native import BASE


def gate_openings(path=None):
    path = BASE.parents[1]/"test/fixtures/vq1_track.json" if path is None else Path(path)
    geometry = json.loads(path.read_text())
    track = _Track()
    track.start(SimpleNamespace(**geometry["handshake"]))
    for chunk in geometry["chunks"]:
        track.receive(SimpleNamespace(seqnr=chunk["seqnr"], data=bytes.fromhex(chunk["data_hex"])), 0.)
    width, height = geometry["opening_width_m"], geometry["opening_height_m"]
    corners = .5*np.array([[-width, 0, -height], [width, 0, -height], [width, 0, height], [-width, 0, height]])
    offset = geometry["pivot_to_center_local_ned_m"]
    return np.array([(corners+offset) @ Quaternion(*gate.orientation).as_rotation_matrix().T+gate.position
                     for gate in track.gates])
