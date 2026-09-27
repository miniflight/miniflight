# Controllers

A controller inherits `BaseController` and implements one method:

```python
from miniflight import BodyRates
from target.aigp.controllers import BaseController

class Controller(BaseController):
    def update(self, state, gate_index):
        return BodyRates(roll_rate=0, pitch_rate=0, yaw_rate=0, thrust=0)
```

Save it as `target/aigp/controllers/mine.py`, then run:

```sh
python target/aigp/simulator.py vq1.r1 --controller mine
```

`state` contains vehicle observations. `gate_index` is the zero-based active gate
reported by the simulator. Return one `BodyRates`, `PositionNed`, or `VelocityNed`
command. `BodyRates` uses rad/s and thrust from 0 to 1. NED position and velocity
use metres and metres per second. Zero thrust does not hover.

A constructor initializes the controller's own memory, such as PID integrals.
The simulator owns sockets, clocks, process lifetime, arming, and disarming.
It calls `update` only after native GO and stops on native finish. Returning `None`
waits for required observations before arming; returning `None` after starting
stops the run. Raise `StopIteration` to stop early.

## observations

`state.acceleration` and `state.gyro` are body-frame IMU samples in m/s² and rad/s.
`state.time` and `state.dt` are simulator seconds; the first `dt` is zero.
Every update has a new IMU sample.

`state.motion` contains NED `position` and `velocity`; `state.attitude` contains
roll, pitch, and yaw in radians. VQ2 does not publish these observations.
`state.frame` holds the latest camera image as `bgr`, with its ID and timestamp.
`state.motors` contains reported output channels and an active mask, not measured RPM.
Unavailable or stale optional observations are `None`. Each sample keeps its own
timestamp; the simulator removes samples older than its configured timeout before
calling the controller. The underlying client retains the original telemetry.

## r1 baseline

`r1_gates` uses VQ1 position telemetry to choose a point one metre beyond each gate.
It keeps that point until the reported gate index advances, then chooses the next.
At index six it holds the final point until the simulator reports native finish.
The pure `gate_target(position, index)` function is reusable by a lower-plane R1
controller. A fresh controller can replay the same `(state, gate_index)` sequence
without a connection or clock.

The default command rate is 50 Hz. Missed ticks are skipped. Loss of fresh IMU
stops control and disarms; the simulator can wait five seconds for a delayed
native finish, without resuming control or rearming.

The controller/simulator boundary follows
[comma's controls_challenge](https://github.com/commaai/controls_challenge/tree/be8edfa849acdccfa2cb0092151ab28590d63c03).

[Setup](../README.md) · [Vehicle API](../docs/vehicle-api.md)
