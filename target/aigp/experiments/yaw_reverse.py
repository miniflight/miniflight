"""Reverse the existing yaw turns to separate direction from turn order."""

from target.aigp.experiments.image_timing import Controller as YawTurns


class Controller(YawTurns):
    yaw_rate = -.25
