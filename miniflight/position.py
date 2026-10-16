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
    resistance_per_m: Vector3 = (0, 0, 0)  # quadratic resistance in the motion frame, 1/m
    body_resistance_per_m: Vector3 | None = None  # optional resistance along FRD body axes, 1/m
    thrust_curve: tuple[float, float] | None = None  # optional F(u) = linear*u + quadratic*u², m/s²
    max_horizontal_acceleration: float | None = None  # otherwise use max_acceleration, m/s²


def quadratic_resistance(velocity, coefficients):
    """Acceleration needed to cancel resistance along each velocity axis, m/s²."""
    if not all(math.isfinite(v) for v in velocity) or not all(math.isfinite(c) and c >= 0 for c in coefficients):
        raise ValueError("resistance requires finite velocity and nonnegative coefficients")
    return tuple(c * abs(v) * v for v, c in zip(velocity, coefficients, strict=True))


def _limit(vector, magnitude):
    scale = min(1.0, magnitude / max(math.hypot(*vector), 1e-12))
    return tuple(value * scale for value in vector)


def trajectory_acceleration(position, velocity, reference, integral, dt, *, kp, ki, kd, integral_limit):
    """Track (position, velocity, acceleration) in one Cartesian frame, in SI units.

    Return requested acceleration and the next integral contribution (m/s²).
    The caller owns state, elapsed time, observation freshness and target selection.
    """
    target_position, target_velocity, feedforward = reference
    error = tuple(wanted - actual for wanted, actual in zip(target_position, position, strict=True))
    integral = tuple(max(-integral_limit, min(integral_limit, previous + ki * dt * e))
                     for previous, e in zip(integral, error, strict=True))
    acceleration = tuple(a + kp * e + i + kd * (wanted - actual)
                         for a, e, i, wanted, actual in zip(feedforward, error, integral, target_velocity, velocity, strict=True))
    return acceleration, integral


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
                         attitude: Vector3, target_yaw: float, *, rate_feedforward: Vector3 = (0, 0, 0)) -> BodyRates:
    """One attitude-feedback update from desired NED acceleration in m/s².

    Attitude is roll/pitch/yaw of FRD body axes relative to local NED. Thrust uses
    the supplied local calibration around hover. The caller owns sample timing,
    freshness, target selection, and any persistent state. No I/O occurs here.
    """
    horizontal_limit = config.max_acceleration if config.max_horizontal_acceleration is None else config.max_horizontal_acceleration
    parameters = (config.hover_thrust, config.thrust_acceleration, config.attitude_gain,
                  config.max_acceleration, horizontal_limit, config.max_tilt, config.max_rate)
    if not all(math.isfinite(value) and value > 0 for value in parameters):
        raise ValueError("position control parameters must be finite and positive")
    if config.hover_thrust >= 1 or config.max_acceleration >= GRAVITY or config.max_tilt >= math.pi / 2:
        raise ValueError("hover thrust, acceleration, or tilt exceeds the upright control range")

    vectors = (acceleration, attitude, rate_feedforward)
    if any(len(vector) != 3 for vector in vectors):
        raise ValueError("position control requires three-component vectors")
    if not math.isfinite(target_yaw) or not all(math.isfinite(value) for vector in vectors for value in vector):
        raise ValueError("position control inputs must be finite")

    down = max(-config.max_acceleration, min(config.max_acceleration, acceleration[2]))
    up = GRAVITY - down
    north, east = _limit(acceleration[:2], min(horizontal_limit, up * math.tan(config.max_tilt)))

    roll, pitch, yaw = attitude
    sr, cr, sp, cp = math.sin(roll), math.cos(roll), math.sin(pitch), math.cos(pitch)
    sy, cy = math.sin(target_yaw), math.cos(target_yaw)
    vertical = cr * cp
    if vertical <= 0:
        raise ValueError("position control requires an upright vehicle")

    # Express the desired thrust direction in the commanded heading frame.
    forward, right = cy * north + sy * east, -sy * north + cy * east
    desired_roll = math.atan2(right, math.hypot(up, forward))
    desired_pitch = math.atan2(-forward, up)
    roll_dot = config.attitude_gain * (desired_roll - roll)
    pitch_dot = config.attitude_gain * (desired_pitch - pitch)
    yaw_dot = config.attitude_gain * ((target_yaw - yaw + math.pi) % (2 * math.pi) - math.pi)

    # Euler angle derivatives are not body rates when the vehicle is tilted.
    feedback = (roll_dot - sp * yaw_dot, cr * pitch_dot + sr * cp * yaw_dot, -sr * pitch_dot + cr * cp * yaw_dot)
    rates = _limit(tuple(f + r for f, r in zip(feedback, rate_feedforward, strict=True)), config.max_rate)
    thrust = config.hover_thrust + (up / vertical - GRAVITY) / config.thrust_acceleration
    if config.thrust_curve is not None:
        linear, quadratic = config.thrust_curve
        if not all(math.isfinite(k) and k >= 0 for k in (linear, quadratic)) or linear + quadratic == 0:
            raise ValueError("thrust curve requires nonnegative finite coefficients and positive force")
        force = up / vertical
        thrust = 2 * force / (linear + math.sqrt(linear * linear + 4 * quadratic * force))
    return BodyRates(*rates, max(0.0, min(1.0, thrust)))
