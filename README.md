# miniflight

A Python API for controlling a vehicle. A controller reads observations and
returns a position, velocity, or body-rate command. The target receives that
command and runs its lower control loops. The included target is the AI-GP
flight simulator.

For example, `r1_gates` chooses a point beyond the next gate and returns
`PositionNed`. VQ1 handles the flight to that point. `r1_body_rates` also computes
position and attitude feedback in Python and returns `BodyRates`; VQ1 handles
angular rates and motor mixing.

On macOS, install the runtime once:

```sh
brew install git-lfs python@3.11
brew install --cask gcenx/wine/game-porting-toolkit
```

From the repository root:

```sh
git lfs install
git lfs pull
python3.11 -m pip install -e ".[aigp]"
python3.11 -m target.aigp.simulator vq1.r1 --controller r1_gates
```

This opens VQ1, waits for race GO, arms the drone, and follows the six R1 gates.
It stops when the simulator reports finish. Ctrl+C stops the controller and
closes its simulator. Use `r1_body_rates` to run the body-rate controller.
See [the arena setup](target/aigp/README.md) for other targets and launch options.

The same run can be started from Python:

```python
from target.aigp.simulator import AIGPSimulator
from target.aigp.controllers.r1_gates import Controller

AIGPSimulator(Controller(), target="vq1.r1").rollout()
```

The runner calls `controller.update(state, gate_index, gates)` and sends the
returned command. It owns the connection, heartbeat, command cadence, arming,
and shutdown. The requested command rate is 50 Hz by default. The controller
owns its target choices and calculations.
Return `None` while waiting for required observations before control starts.

`state` contains physical observations:

| field | value |
| --- | --- |
| `time`, `dt` | IMU time and interval since the previous read, in seconds |
| `acceleration` | `(x, y, z)` body acceleration, in m/s² |
| `gyro` | `(x, y, z)` body angular rates, in rad/s |
| `motion` | local NED position in metres and velocity in m/s |
| `attitude` | roll, pitch, and yaw angles, in radians |
| `motors` | reported output channels and their active bitmask |
| `frame` | a camera frame with a read-only `(H, W, 3)` uint8 BGR array |

Motion, attitude, motors, and camera data are optional. Each keeps its own device
timestamp and host receipt time. Missing or stale records are `None` in
controller updates; the observations are not one synchronized estimate.
Motor output units belong to the target.

`gate_index` identifies the active gate. `gates` is the cached geometry for the
whole track, with each gate's center, orientation, width, and height. It remains
available between track transfers. It is `None` until a complete usable track
arrives.

A controller returns one of these commands:

| command | request | feedback handled by VQ1 |
| --- | --- | --- |
| `PositionNed(north, east, down)` | local position, in metres | position and lower loops |
| `VelocityNed(north, east, down)` | local velocity, in m/s | velocity and lower loops |
| `BodyRates(roll_rate, pitch_rate, yaw_rate, thrust)` | body rates in rad/s, collective thrust from 0 to 1 | angular rates and motor mixing |

NED means north, east, down. Body axes are forward, right, down. Body rates are
rotation speeds, not roll, pitch, and yaw angles. A fixed position request lets
VQ1 hold position. Zero body rates do not level a tilted drone, and zero thrust
does not hover.

[Write a controller](target/aigp/controllers/README.md) ·
[Vehicle API](target/aigp/docs/vehicle-api.md) ·
[AI-GP specification](target/aigp/docs/VQ1-Technical-Specification-00.02.pdf)
