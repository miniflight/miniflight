# Controllers

Write one file in this directory and export `Controller(BaseController)`:

```python
from miniflight import BodyRates
from target.aigp.controllers import BaseController

class Controller(BaseController[BodyRates]):
    def update(self, telemetry, frames, race_state):
        return BodyRates()
```

Run from `target/aigp`: `python simulator.py vq1.r1 --controller mine`.
The runner constructs `Controller()` after the native heartbeat.

`telemetry` is an ordered tuple of new `Packet` arrivals from UDP 14550.
`packet.data` is the original pymavlink message, with full fields, flags and
source timestamps. `packet.received_at` is host monotonic receipt time.
`packet.decoded` contains wire `RaceTelemetry`, `TrackInfo(transfer_id, gates)` when a
course transfer completes, or `None`. Each `Gate` retains the published NED
mesh pivot, wxyz quaternion and overall dimensions. `packet.privileged` labels pose
(`ATTITUDE`, `LOCAL_POSITION_NED`, `ODOMETRY`) and course traffic.
The receiver preserves redacted fields; the controller decides if they are useful.
`race_state` is the latest `NativeRaceState` snapshot from our
[Lua instrumentation](../config/race.lua). This additional privileged stream
reads engine GO, validity, completion and gate index. Its `time_seconds` is game
race time; `received_at` is Python's file-read time. Each snapshot retains mesh
bounds centers read once at track load, converted from world centimetres to NED
metres relative to the starting grid. The R1 controllers use these centers and
retain the published gate packets unchanged.

`frames` is a tuple of new complete images from UDP 5600. Each has `id`, device
`time_ns`, host `received_at` and readonly `bgr` (`uint8[height,width,3]`).
Either tuple may be empty. Each stream keeps its own timing.
Store history, convert coordinates, and estimate state in the controller.

Inputs are delivered before GO and at native finish. Commands are sent only
after GO with fresh IMU. Return `BodyRates`, `PositionNed`, `VelocityNed`, `MotorControls`, or `None`
while waiting for required startup input. Returning `None` after arming ends the
run; `StopIteration` stops early. A race reset ends the run. IMU loss disarms and
starts a bounded finish wait; recovering IMU never resumes control.

`r1_gates` chooses a point beyond the active native mesh center. `r1_body_rates`
reuses that planner and applies the numerical `position_control` routine.
`r1_motors` keeps that outer controller, then applies rate PI and the measured
motor mixer. Its integral uses the IMU clock; repeated samples do not add to it.
Use `AIGPSimulator(Controller, timeout=.3)` for the tested 50 Hz motor path.
A device-time gap above 0.3 seconds, collision, or infeasible motor output stops
the run. Motor limits cause an error; this controller does not clip and continue.
`r1_beautiful` uses PID feedback and accepts a timed position/velocity/acceleration
reference. Without a reference it follows the next gate center.
VQ2 needs its own perception and estimation when course/pose streams are blocked.

[Setup](../README.md) · [Native wiring](../docs/wiring.md) · [Recording](../experiments/README.md)
