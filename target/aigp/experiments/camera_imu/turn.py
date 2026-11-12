"""A scheduled velocity bend for sensor-only estimator comparisons."""

import numpy as np

from miniflight.position import acceleration_control, quadratic_resistance
from . import controller as base


class GateController(base.GateController):
    def drive(self, target):
        settings = base.SETTINGS
        elapsed = None if self.began is None else self.inertial.time-self.began
        turning = (elapsed is not None and settings['turn_start_s'] <= elapsed
                   < settings['turn_start_s']+settings['turn_duration_s'])
        if self.signal:
            self.signal['turn_test'] = dict(active=turning, elapsed_since_lock_s=elapsed)
        if not turning:
            return super().drive(target)
        angle = np.radians(settings['turn_angle_deg'])*settings['turn_sign']
        sideways = np.cross([0., 0., 1.], self.direction)
        wanted = self.config.max_speed*(np.cos(angle)*self.direction+np.sin(angle)*sideways)
        wanted[2] = np.clip(self.config.position_gain*(self.anchor[2]-self.inertial.position[2]), -.4, .4)
        body = self.inertial.rotation.as_rotation_matrix()
        acceleration = self.config.velocity_gain*(wanted-self.inertial.velocity)
        acceleration += body @ quadratic_resistance(body.T @ self.inertial.velocity, self.config.body_resistance_per_m)
        self.signal['turn_test']['desired_velocity'] = wanted.tolist()
        return acceleration_control(self.config, acceleration, self.inertial.rotation.to_euler(), 0.)


class Controller(base.Controller):
    def __init__(self):
        self.gate = GateController()
        self.motor = base.single.Motors(position=self.gate)
