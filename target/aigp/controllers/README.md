# Controllers

A controller inherits `BaseController` and implements one method:

```python
from miniflight import BodyRates
from target.aigp.controllers import BaseController

class Controller(BaseController[BodyRates]):
    def update(self, state, gate_index, gates):
        return BodyRates(roll_rate=0, pitch_rate=0, yaw_rate=0, thrust=0)
```

Save it as `target/aigp/controllers/mine.py`, then run from `target/aigp`:

```sh
python simulator.py vq1.r1 --controller mine
```

`state` contains vehicle observations. `gate_index` is the zero-based active gate
reported by the simulator. `gates` is an immutable tuple of `Gate` values, or
`None` until a complete usable track arrives. Each gate exposes:

- `id`: the published gate index.
- `origin`: the published gate base in NED metres. Older recordings omit it.
- `center`: the derived opening center in NED metres.
- `orientation`: the published quaternion in wxyz order, normalized to unit length.
- `width`, `height`: reported overall bounds in metres, not opening clearance.

Each update gets a new IMU sample and the cached whole track. Motion, attitude,
motors and camera images can come from slower streams; each keeps its own
timestamp. The gate tuple stays available between transfers. An incomplete
replacement does not replace the current track.

Captured VQ1 packets report about 2.72 m bounds. Section 3.7 of the
[VQ1 specification](../docs/VQ1-Technical-Specification-00.02.pdf) gives a separate
1.5 m inner opening; that opening size is not a field in the track packet.
The gate-local axis normal to the opening still needs verification. A gate's
orientation is not directly a desired drone attitude or a crossing direction.

Return one `BodyRates`, `PositionNed`, or `VelocityNed` command. `BodyRates` uses
rad/s and thrust from 0 to 1. NED position and velocity use metres and metres per
second. Zero thrust does not hover.

The type parameter declares the controller's output plane. Use
`BaseController[PositionNed]`, `BaseController[VelocityNed]`, or
`BaseController[BodyRates]` for a single plane. A routine that intentionally
switches planes can use a union, such as
`BaseController[BodyRates | PositionNed]` in the body-rate probe. This is a typing
contract; `Vehicle.validate()` checks the actual returned command against the
target's supported commands before arming or sending. All planes use the same
`update` method and runner. The target owns wire encoding and coordinate conversion;
the controller owns any deliberate transition between command planes.

A constructor initializes the controller's own memory, such as PID integrals.
Create a fresh controller for each flight. Each `AIGPSimulator` instance is single-use;
reusing one raises before touching its connection, even after a failed run.
The simulator owns sockets, clocks, process lifetime, arming, and disarming.
It calls `update` only after native GO and stops on native finish. Returning `None`
waits for required observations before arming; returning `None` after starting
stops the run. Raise `StopIteration` to stop early.

The simulator rejects race resets and gate indices outside the published track.
With track geometry available, each gate must be passed within 45 simulator seconds
of the first command for that gate. Set `AIGPSimulator(..., gate_timeout=90)` to allow
more time. After the last gate, control continues until native finish.

## observations

`state.acceleration` and `state.gyro` are body-frame IMU samples in m/s² and rad/s.
`state.time` and `state.dt` are simulator seconds; the first `dt` is zero.
Every update has a new IMU sample.

`state.motion` contains NED `position` and `velocity`; `state.attitude` contains
roll, pitch, and yaw in radians. VQ2 does not publish these observations.
`state.frame` holds the latest camera image as `bgr`, with its ID and timestamp.
`state.motors` contains reported output channels and an active mask, not measured RPM.
Unavailable or stale optional observations are `None`; motion with nonfinite
position or velocity and attitude with nonfinite angles are also unavailable. Each sample keeps its own
timestamp; the simulator removes samples older than its configured timeout before
calling the controller. The underlying client retains the original telemetry.

## r1 baseline

`r1_gates` uses the published track and VQ1 position telemetry to choose a point
one metre beyond each gate. It keeps that point until the reported gate index
advances, then chooses the next. After the last gate it holds the final point until
native finish. The controller has no stored course coordinates or fixed gate count.

The simulator assembles track packets and retains each gate's published origin.
It derives the opening center with `origin + rotate(orientation, (0, 0, -height / 2))`.
It publishes only a complete track, retaining the last one during an incomplete
replacement. Missing or nulled geometry remains unavailable; there is no fallback
map. Course data is separate from generic vehicle state and does not expire on an
IMU timeout.

Start R1 through the simulator so the receiver is listening when track data is
published. A late attach can miss that transfer and leave the controller waiting
for geometry without arming.

`Gate` is part of the controller input contract in `controllers/__init__.py`.
The gate-selection rule is `r1_gates.gate_target(position, index, gates)`.
It returns a `Ned` position. `r1_gates` sends that position as a `PositionNed`
command. `r1_body_rates` uses the same function, then computes the body rates and
thrust needed to reach the position. Both routines follow the same gate progress
and finish conditions; they use different command planes.

A fresh controller can replay `(state, gate_index, gates)` samples without a
connection or clock. Optional [recording and replay tools](../experiments/README.md#recording-and-replay)
are kept with the experiments; the simulator and controllers do not import them.

## r1 body rates

`r1_body_rates` uses the same gate-target policy and closes position and attitude
feedback in Python. It requires VQ1 motion, attitude, and the published course;
missing required observations return `None`. Every flight command is `BodyRates`.

The routine stores its desired position as `target_position` and its desired
heading as `target_yaw`. It deliberately holds the initial heading throughout
the course. The position target changes only when the native gate index advances,
and the final target is held until native finish.

```sh
python simulator.py vq1.r1 --controller r1_body_rates
```

The numerical function is `miniflight.position.position_control`: position error
becomes a bounded velocity target, velocity error becomes a bounded acceleration,
and the desired thrust direction becomes attitude feedback and body-rate commands.
It is stateless PD, so it needs no integral, timestep, or hidden numerical memory.
Its inputs are fixed-size numerical values; it does not receive a `Vehicle`, race
state, camera pixels, or transport. `PositionConfig` carries gains, limits, and
the vehicle's local hover/thrust calibration. The VQ1 values live in the AIGP
controller, not in the generic numerical function.

The returned command is the numerical controller's output. The separate
[yaw tracking experiment](../docs/yaw-tracking.md) tests sustained rate accuracy
through the probe's `--yaw-feedback` option; its additional gyro correction is
not part of this gate routine.

The simulator still owns angular-rate stabilization and motor mixing. This
controller is not a VQ2 estimator or a hardware-validated flight stack.

The default command rate is 50 Hz; configured rates must be positive and below
100 Hz, as required by the bundled specification. Missed ticks are skipped. Loss of fresh IMU
stops control and disarms; the simulator can wait five seconds for a delayed
native finish, without resuming control or rearming.

The adapter converts AIGP's angular wire signs into the API's FRD/NED convention.
See the [body-rate measurements](../docs/body-rates.md) before writing a lower-level controller.

The controller/simulator boundary follows
[comma's controls_challenge](https://github.com/commaai/controls_challenge/tree/be8edfa849acdccfa2cb0092151ab28590d63c03).

## r1 trpy

`r1_trpy` is being built one step at a time. It currently computes desired NED
acceleration from position error and velocity. TRPY means thrust and roll, pitch,
yaw angles; it is a different output plane from `BodyRates`. The attitude-and-thrust
command is not exposed yet. Its `update` method raises
`NotImplementedError`; acceleration-to-command conversion is not implemented yet.

[Setup](../README.md) · [Vehicle API](../docs/vehicle-api.md)
