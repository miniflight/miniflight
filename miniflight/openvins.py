"""Optional OpenVINS binding. Inputs are camera images and paired IMU samples."""

import ctypes
import math
from pathlib import Path
from typing import NamedTuple

import cv2
import numpy as np

from miniflight.state import Attitude, Frame, ImuSample, Motion, Ned


class Estimate(NamedTuple):
    motion: Motion
    attitude: Attitude


class OpenVINS:
    """Estimate motion in a local z-down frame with a free origin and heading.

    The first second establishes a receipt-clock relation between camera and
    IMU. OpenVINS then estimates the remaining camera–IMU time offset. No pose,
    velocity, gate information or control reference enters this observer.
    """

    def __init__(self, library, config):
        self.lib = ctypes.CDLL(str(Path(library).resolve()))
        pointer, seconds = ctypes.c_void_p, ctypes.c_double
        vector = np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS")
        image = np.ctypeslib.ndpointer(dtype=np.uint8, flags="C_CONTIGUOUS")
        self.lib.vio_create.argtypes, self.lib.vio_create.restype = [ctypes.c_char_p], pointer
        self.lib.vio_destroy.argtypes = [pointer]
        self.lib.vio_destroy.restype = None
        self.lib.vio_imu.argtypes = [pointer, seconds, vector, vector]
        self.lib.vio_imu.restype = ctypes.c_int
        self.lib.vio_camera.argtypes = [pointer, seconds, image, ctypes.c_int, ctypes.c_int]
        self.lib.vio_camera.restype = ctypes.c_int
        self.lib.vio_state.argtypes, self.lib.vio_state.restype = [pointer, vector], ctypes.c_int
        self.config = str(Path(config).resolve()).encode()
        self.handle = self.clock = self.estimate = self.raw = self.received_at = None
        self.pending = []

    def close(self):
        if self.handle:
            self.lib.vio_destroy(self.handle)
            self.handle = None

    def __del__(self):
        if getattr(self, "handle", None):
            self.close()

    def update(self, samples: tuple[ImuSample, ...], frames: tuple[Frame, ...]):
        arrivals = sorted((*samples, *frames), key=lambda sample: sample.received_at)
        if not arrivals:
            return self.estimate
        if self.clock is None:
            self.pending.extend(arrivals)
            if self.pending[-1].received_at - self.pending[0].received_at < 1:
                return None
            imu = [sample for sample in self.pending if isinstance(sample, ImuSample)]
            cameras = [sample for sample in self.pending if isinstance(sample, Frame)]
            if not imu or not cameras:
                return None
            camera_epoch = cameras[0].time_ns
            imu_offset = float(np.median([sample.received_at - sample.time for sample in imu]))
            camera_origin = float(np.median([sample.received_at - (sample.time_ns - camera_epoch) * 1e-9 for sample in cameras]))
            self.clock = camera_epoch, imu_offset, camera_origin
            self.handle = self.lib.vio_create(self.config)
            if not self.handle:
                raise RuntimeError("OpenVINS initialization failed")
            arrivals, self.pending = self.pending, []
        camera_epoch, imu_offset, camera_origin = self.clock
        for sample in arrivals:
            if isinstance(sample, ImuSample):
                accepted = self.lib.vio_imu(self.handle, sample.time, np.array(sample.gyro), np.array(sample.acceleration))
                self.received_at = sample.received_at
            else:
                gray = np.ascontiguousarray(cv2.cvtColor(sample.bgr, cv2.COLOR_BGR2GRAY))
                timestamp = (sample.time_ns - camera_epoch) * 1e-9 + camera_origin - imu_offset
                accepted = self.lib.vio_camera(self.handle, timestamp, gray, gray.shape[1], gray.shape[0])
            if not accepted:
                raise RuntimeError("OpenVINS rejected a sensor measurement")
        data = np.zeros(25)
        if not self.lib.vio_state(self.handle, data):
            return self.estimate
        if not np.isfinite(data).all():
            raise ValueError("nonfinite OpenVINS state")
        self.raw = data  # time, camera time, p(3), v(3), R(9), gyro bias(3), accel bias(3), offset, clone count
        z_down = np.diag([1., -1., -1.])
        rotation = z_down @ data[8:17].reshape(3, 3)
        position, velocity = z_down @ data[2:5], z_down @ data[5:8]
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        pitch = math.atan2(-rotation[2, 0], math.hypot(rotation[2, 1], rotation[2, 2]))
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
        self.estimate = Estimate(Motion(data[0], self.received_at, Ned(*position), Ned(*velocity)),
                                 Attitude(data[0], self.received_at, roll, pitch, yaw))
        return self.estimate
