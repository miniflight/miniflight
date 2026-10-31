# Controllers

Write one file in this directory and export `Controller(BaseController)`:

```python
from miniflight import BodyRates, vehicle
from target.aigp.controllers import BaseController

class Controller(BaseController[BodyRates]):
    def update(self, telemetry, frames, race_state):
        if vehicle.state is None or vehicle.state.gyro is None:
            return None
        return BodyRates()
```

Run from the repository root with `python -m target.aigp.simulator vq1.r1 --controller mine`.
The runner also accepts a module outside this directory, for example
`--controller examples.aigp.velocity_ned`. It constructs `Controller()` after the
native heartbeat. Recording and process replay use the same module loader.

The runner calls `vehicle.read()` once per tick. Read `vehicle.state` in the
controller: the target supplies accepted IMU, motion, attitude, motor feedback
and the latest frame. Motion uses NED; angular observations use FRD. Each sample
retains its own timestamp. Motor channels and image pixels retain reported values.
Unavailable samples are `None`; `state.time` and `state.dt` belong to the IMU.
Keep controller history, such as integrals; do not keep another sensor cache.
`state.imu_samples` contains every accepted accelerometer/gyro pair in this read,
in arrival order. `state.acceleration` and `state.gyro` retain the latest pair.

`telemetry` is an ordered tuple of new `Packet` arrivals from UDP 14550.
`packet.data` is the original pymavlink message, with full fields, flags and
source timestamps. `packet.received_at` is host monotonic receipt time.
`packet.decoded` contains an `ImuSample`, `Motion`, `Attitude`, wire `RaceTelemetry`,
`TrackInfo(transfer_id, gates)` when a course transfer completes, or `None`.
Each `Gate` retains the published NED
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
Raw arrivals remain available for AIGP course/lifecycle events and recording.

The runtime rejects native collision reports before calling the controller.
Inputs are delivered before GO and at native finish. Commands are sent only
after GO with fresh rate feedback. Return `BodyRates`, `PositionNed`, `VelocityNed`, `MotorControls`, or `None`
while waiting for required startup input. Returning `None` after arming ends the
run; `StopIteration` stops early. A race reset ends the run. Rate feedback loss disarms and
starts a bounded finish wait; recovering IMU never resumes control.

`r1_gates` chooses a point beyond the active native mesh center. `r1_body_rates`
reuses that planner and applies the numerical `position_control` routine.
`r1_motors` keeps that outer controller, then applies rate PI and the measured
motor mixer. Its integral uses the selected rate sample's clock; repeated samples do not add to it.
Use `AIGPSimulator(Controller, timeout=.3)` for the tested 50 Hz motor path.
A device-time gap above 0.3 seconds or nonfinite motor output stops the run.
Motor inputs are not clipped to 0–1; their scale and limits belong to the target.
`r1_beautiful` uses PID feedback and accepts a timed position/velocity/acceleration
reference. It registers one native initial heading, then uses one IMU rotation
estimate for roll, pitch and yaw. Without a reference it follows the next gate
center. The reference experiment supplies this follower to `r1_motors`.
VQ2 needs its own perception and estimation when course/pose streams are blocked.

[Setup](../README.md) · [Native wiring](../docs/wiring.md) · [Recording](../experiments/README.md)
