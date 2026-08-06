"""Small pinhole camera geometry."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Tuple


Vector2 = Tuple[float, float]
Vector3 = Tuple[float, float, float]
Matrix3 = Tuple[Vector3, Vector3, Vector3]


@dataclass(frozen=True)
class PinholeCamera:
    """Map pixels and one known vertical size into the body frame."""

    focal_x_px: float
    focal_y_px: float
    center_x_px: float
    center_y_px: float
    camera_to_body: Matrix3

    def __post_init__(self) -> None:
        values = (
            self.focal_x_px,
            self.focal_y_px,
            self.center_x_px,
            self.center_y_px,
            *(value for row in self.camera_to_body for value in row),
        )
        if self.focal_x_px <= 0.0 or self.focal_y_px <= 0.0:
            raise ValueError("camera focal lengths must be positive")
        if len(self.camera_to_body) != 3 or any(
            len(row) != 3 for row in self.camera_to_body
        ):
            raise ValueError("camera rotation must be 3 by 3")
        if not all(math.isfinite(value) for value in values):
            raise ValueError("camera values must be finite")

    def measure_body_vector(
        self,
        center_px: Vector2,
        vertical_span_px: float,
        vertical_span_m: float,
    ) -> Vector3:
        """Return the body-frame vector to a known vertical object."""
        if vertical_span_px <= 0.0 or vertical_span_m <= 0.0:
            raise ValueError("object spans must be positive")
        depth_m = self.focal_y_px * vertical_span_m / vertical_span_px
        camera_vector = (
            depth_m * (center_px[0] - self.center_x_px) / self.focal_x_px,
            depth_m * (center_px[1] - self.center_y_px) / self.focal_y_px,
            depth_m,
        )
        return self._rotate(self.camera_to_body, camera_vector)

    def project_body_vector(self, body_vector_m: Vector3) -> Tuple[float, float, float]:
        """Return image coordinates and optical-axis depth."""
        body_to_camera = tuple(zip(*self.camera_to_body))
        camera_vector = self._rotate(body_to_camera, body_vector_m)
        if camera_vector[2] <= 0.0:
            raise ValueError("body vector is behind the camera")
        return (
            self.center_x_px + self.focal_x_px * camera_vector[0] / camera_vector[2],
            self.center_y_px + self.focal_y_px * camera_vector[1] / camera_vector[2],
            camera_vector[2],
        )

    def camera_vector_to_body(self, camera_vector_m: Vector3) -> Vector3:
        """Rotate one camera-frame vector into the body frame."""
        return self._rotate(self.camera_to_body, camera_vector_m)

    @staticmethod
    def _rotate(matrix: Matrix3, vector: Vector3) -> Vector3:
        return tuple(
            sum(row[index] * vector[index] for index in range(3))
            for row in matrix
        )
