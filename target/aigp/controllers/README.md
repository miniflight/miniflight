# Controllers

Write one file in this directory and export `Controller(BaseController)`:

```python
from miniflight import BodyRates
from target.aigp.controllers import BaseController

class Controller(BaseController[BodyRates]):
    def update(self, telemetry, frames):
        return BodyRates()
```

Run from `target/aigp`: `python simulator.py vq1.r1 --controller mine`.
The runner constructs the controller after the native heartbeat. Its constructor
receives `track=None` or the course already received. Do not wait for geometry.

`telemetry` is an ordered tuple of new `Packet` arrivals from UDP 14550.
`packet.data` is the original pymavlink message, with full fields, flags and
source timestamps. `packet.received_at` is host monotonic receipt time.
`packet.decoded` contains a native `RaceStatus` for race packets, a tuple of
`Gate` records when a course transfer completes, or `None` otherwise. Gate
records retain the published NED base, wxyz quaternion and overall dimensions.
The receiver preserves redacted fields; the controller decides if they are useful.

`frames` is a tuple of new complete images from UDP 5600. Each has `id`, device
`time_ns`, host `received_at` and readonly `bgr` (`uint8[height,width,3]`).
Both tuples may be empty. They are independent arrivals, not a synchronized step.
Store history in the controller. Coordinate conversion and estimation are explicit
controller math; absent native streams are not replaced with estimates.

Inputs are delivered before GO and at native finish. Commands are sent only
after GO with fresh IMU. Return `BodyRates`, `PositionNed`, `VelocityNed`, or `None`
while waiting for required startup input. Returning `None` after arming ends the
run; `StopIteration` stops early. A race reset ends the run. IMU loss disarms and
starts a bounded finish wait; recovering IMU never resumes control.

`r1_gates` chooses a point beyond the active published gate. `r1_body_rates`
reuses that planner and applies the numerical `position_control` routine.
VQ2 needs its own perception and estimation when course/pose streams are blocked.
`r1_trpy` remains an unimplemented experiment.

[Setup](../README.md) · [Native wiring](../docs/wiring.md) · [Recording](../experiments/README.md)
