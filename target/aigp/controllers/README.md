# Controllers

Run the simulator and controller together:

```sh
./target/aigp/run vq1.r1 --controller r1_gates
```

`r1_gates` follows the six R1 gate centers using VQ1's built-in position control.
It targets one metre beyond each center and advances on the reported gate index.
After the last gate it holds that target until the native finish signal.
It requires VQ1 position telemetry; VQ2 does not publish it, including on R1.

Copy `target/aigp/controllers/zero.py` to `target/aigp/controllers/mine.py`, then select it by name:

```sh
./target/aigp/run vq2.r2 --controller mine
```

```python
from miniflight import BodyRates, State
from target.aigp.controllers import BaseController

class Controller(BaseController):
    def update(self, state: State) -> BodyRates:
        return BodyRates(roll_rate=0, pitch_rate=0, yaw_rate=0, thrust=0)
```

Rates are rad/s. Thrust is 0–1. `zero` sends zero thrust; it does not hover.
Return `PositionNed(north, east, down)` for absolute position commands in metres.
Return `VelocityNed(north, east, down)` for velocity commands in m/s.
Return `None` to wait for required telemetry before arming; raise `StopIteration`
to stop control. Controllers do not open sockets, launch processes, or manage timing.

The runner launches the simulator, waits for fresh race telemetry to report GO,
then arms and runs the controller. Countdown packets do not permit movement.
Native finish, Ctrl+C, reset, or a controller error ends the run with zero thrust
and disarm. It never takes over an existing simulator.

`self.vehicle` reads and commands the vehicle through `self.client`, the shared
`SimulatorClient`. `update` receives the same snapshot as `self.vehicle.state`.
Return a command from `update`; the runner validates and sends it through the vehicle.

`state.acceleration` and `state.gyro` are the latest body-frame IMU sample
(m/s² and rad/s). `state.time` and `state.dt` are simulator seconds; the first
`dt` is zero. Every update has a new IMU sample. `state.received_at` is that
sample's host-monotonic receipt time.

`state.frame` is the latest complete camera frame, or `None`. It holds `bgr`,
`id`, `time_ns` from the simulator, and host-monotonic `received_at`. Frames
can repeat across updates.

`state.motion` holds local NED `position` in metres and `velocity` in m/s with
named `north`, `east`, `down` components. `state.attitude` holds `roll`, `pitch`,
`yaw` in radians. Both are `None` on VQ2. `state.motors` holds reported output
channels and their active mask, not measured RPM. Each has its own device `time`
and host `received_at`; a fresh IMU does not make the other samples fresh.

`self.race` contains native race signals, including gate progress and finish.
Raw diagnostics remain on `self.client.telemetry`, `self.client.received_at`,
and `self.client.messages`. No observations are synthesized.

The default loop rate is 50 Hz (`--hz 50`), with a 2 Hz heartbeat.
After GO, race packet gaps retain the last confirmed phase and gate index.
Only the native finish value completes the race. A one-second IMU outage stops
control and disarms, then allows five seconds to receive a delayed finish.
If none arrives the run fails. Control never resumes during that finish wait.
Slow updates skip ticks.
Use one controller at a time: MAVLink uses local UDP 14550, camera uses 5600.

[Setup](../README.md) · [Vehicle API](../docs/vehicle-api.md) · [Race event chain](../docs/race-lifecycle.md)
