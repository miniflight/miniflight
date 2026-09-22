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
from miniflight import Control, State

class Controller:
    def update(self, state: State) -> Control:
        return Control(roll_rate=0, pitch_rate=0, yaw_rate=0, thrust=0)
```

Rates are rad/s. Thrust is 0–1. `zero` sends zero thrust; it does not hover.
Return `PositionNed(north, east, down)` for absolute position commands in metres.
Return `None` to wait for required telemetry before arming; raise `StopIteration`
to finish. Controllers do not open sockets, launch processes, or manage timing.

The runner launches the simulator, waits for fresh race telemetry to report GO,
then arms and runs the controller. Countdown packets do not permit movement.
Finish, Ctrl+C, reset, stale telemetry, or an error sends zero thrust and disarm,
then stops the simulator it launched. It never takes over an existing simulator.

The shared target is `target.aigp.SimulatorClient`. It connects to the running
simulator; version and round are selected by the launch command.

`state.acceleration` and `state.gyro` are the latest body-frame IMU sample
(m/s² and rad/s). `state.time` and `state.dt` are simulator seconds; the first
`dt` is zero. Every update has a new IMU sample.

`state.frame` is the latest complete camera frame, or `None`. It holds `bgr`,
`id`, `time_ns` from the simulator, and host-monotonic `received_at`. Frames
can repeat across updates. `state.race` contains the simulator's race fields.

`state.telemetry` holds the latest raw MAVLink messages by name;
`state.received_at` holds their host-monotonic receipt times. For example,
`state.telemetry.get("LOCAL_POSITION_NED")` returns a received position message
or `None`. VQ2 gets no fabricated pose or extra track information.
`state.messages` contains packets received since the previous update, capped
at 2048; this includes IMU samples, collisions and track-data packets.

The default loop rate is 50 Hz (`--hz 50`), with a 2 Hz heartbeat.
Missing IMU or race data for one second stops the runner. Slow updates skip ticks.
Use one controller at a time: MAVLink uses local UDP 14550, camera uses 5600.

[Setup](../README.md) · [Race event chain](../docs/race-lifecycle.md)
