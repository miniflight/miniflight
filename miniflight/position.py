"""Position feedback in NED, producing FRD body rates and collective thrust."""

from dataclasses import dataclass
import math

from miniflight.control import BodyRates


Vector3 = tuple[float, float, float]
GRAVITY = 9.81


@dataclass(frozen=True)
class PositionConfig:
    hover_thrust: float
    thrust_acceleration: float  # local acceleration slope, m/s² per unit thrust
    position_gain: float = 1.0  # position error -> velocity, 1/s
    velocity_gain: float = 2.0  # velocity error -> acceleration, 1/s
    attitude_gain: float = 4.0  # Euler angle error -> Euler rate, 1/s
    max_speed: float = 4.0  # m/s, vector magnitude
    max_acceleration: float = 3.0  # m/s², horizontal magnitude and vertical bound
    max_tilt: float = .35  # radians from vertical
    max_rate: float = .75  # rad/s, body-rate vector magnitude


def _limit(vector, magnitude):
    scale = min(1.0, magnitude / max(math.hypot(*vector), 1e-12))
    return tuple(value * scale for value in vector)


def position_control(config: PositionConfig, position: Vector3, velocity: Vector3,
                     attitude: Vector3, target_position: Vector3, target_yaw: float) -> BodyRates:
    """Position PD followed by attitude feedback; SI units and FRD/NED axes."""
    if not all(math.isfinite(value) and value > 0
               for value in (config.position_gain, config.velocity_gain, config.max_speed)):
        raise ValueError("position gains and speed limit must be finite and positive")
    vectors = (position, velocity, target_position)
    if any(len(vector) != 3 for vector in vectors) or not all(math.isfinite(v) for vector in vectors for v in vector):
        raise ValueError("position control requires finite three-component vectors")
    desired_velocity = _limit(tuple(config.position_gain * (target - actual)
                                    for target, actual in zip(target_position, position)), config.max_speed)
    acceleration = tuple(config.velocity_gain * (desired - actual)
                         for desired, actual in zip(desired_velocity, velocity))
    return acceleration_control(config, acceleration, attitude, target_yaw)


def acceleration_control(config: PositionConfig, acceleration: Vector3,
                         attitude: Vector3, target_yaw: float) -> BodyRates:
    """One attitude-feedback update from desired NED acceleration in m/s².

    Attitude is roll/pitch/yaw of FRD body axes relative to local NED. Thrust uses
    the supplied local calibration around hover. The caller owns sample timing,
    freshness, target selection, and any persistent state. No I/O occurs here.
    """
    parameters = (config.hover_thrust, config.thrust_acceleration, config.attitude_gain,
                  config.max_acceleration, config.max_tilt, config.max_rate)
    if not all(math.isfinite(value) and value > 0 for value in parameters):
        raise ValueError("position control parameters must be finite and positive")
    if config.hover_thrust >= 1 or config.max_acceleration >= GRAVITY or config.max_tilt >= math.pi / 2:
        raise ValueError("hover thrust, acceleration, or tilt exceeds the upright control range")

    vectors = (acceleration, attitude)
    if any(len(vector) != 3 for vector in vectors):
        raise ValueError("position control requires three-component vectors")
    if not math.isfinite(target_yaw) or not all(math.isfinite(value) for vector in vectors for value in vector):
        raise ValueError("position control inputs must be finite")

    down = max(-config.max_acceleration, min(config.max_acceleration, acceleration[2]))
    up = GRAVITY - down
    north, east = _limit(acceleration[:2], min(config.max_acceleration, up * math.tan(config.max_tilt)))

    roll, pitch, yaw = attitude
    sr, cr, sp, cp = math.sin(roll), math.cos(roll), math.sin(pitch), math.cos(pitch)
    sy, cy = math.sin(yaw), math.cos(yaw)
    vertical = cr * cp
    if vertical <= 0:
        raise ValueError("position control requires an upright vehicle")

    # Express the desired thrust direction in the current heading frame.
    forward, right = cy * north + sy * east, -sy * north + cy * east
    desired_roll = math.atan2(right, math.hypot(up, forward))
    desired_pitch = math.atan2(-forward, up)
    roll_dot = config.attitude_gain * (desired_roll - roll)
    pitch_dot = config.attitude_gain * (desired_pitch - pitch)
    yaw_dot = config.attitude_gain * ((target_yaw - yaw + math.pi) % (2 * math.pi) - math.pi)

    # Euler angle derivatives are not body rates when the vehicle is tilted.
    rates = _limit((roll_dot - sp * yaw_dot,
                    cr * pitch_dot + sr * cp * yaw_dot,
                    -sr * pitch_dot + cr * cp * yaw_dot), config.max_rate)
    thrust = config.hover_thrust + (up / vertical - GRAVITY) / config.thrust_acceleration
    return BodyRates(*rates, max(0.0, min(1.0, thrust)))
