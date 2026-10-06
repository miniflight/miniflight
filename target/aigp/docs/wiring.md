# simulator wiring

The entrypoint is [simulator.py](../simulator.py). The [specification](VQ1-Technical-Specification-00.02.pdf),
sections 3 and 4, defines the physical frames, command messages, timing, and camera
packet. The [vendor example](reference/PyAIPilotExample-v4/) supplies the race and
track payloads and the body-rate extension. This note maps those interfaces to the
current VQ1 build 3391 path; it does not infer VQ2 sensor availability from VQ1.

## launch and native start

`main` selects a target and controller. `AIGPSimulator.rollout` reserves the client
ports, enters `launch`, and owns the control loop. `launch` checks the simulator
ports, locks its Wine prefix, calls `prepare`, and starts `DCGame-Win64-Shipping.exe`
in the selected master level. `prepare` verifies the archive, extracts it once,
and copies the three files from `config/<version>` into the installed simulator.

For VQ1, [main.lua](../config/vq1/main.lua) executes this native startup sequence:

```text
MAP_anduril_master + GameModeRaceBase
  -> stream MAP_anduril_lighting_DAY -> OnLightingLevelLoaded
  -> stream MAP_anduril_track01 -> OnTrackLevelLoaded
  -> valid track and starting grid -> assign StartingGridSlot
  -> OpponentsPopup.ConfirmPopup
  -> ServerForceStartRaceWithoutRaceVerification
  -> GameStateRaceBase.HasRaceStarted
```

Lua drives level and race setup. The executable publishes the scheduled start,
current boot time, gate progress, and finish. Python waits for the native GO packet;
`physics_ready` alone does not start the controller.

## the two UDP connections

| Connection | Simulator endpoint | Python endpoint | Data |
| --- | --- | --- | --- |
| MAVLink 2 | The heartbeat's sender, normally `127.0.0.1:14560` | `127.0.0.1:14550` | Commands, IMU, optional pose/motors, race, track |
| Camera | Normally `127.0.0.1:5601` | `127.0.0.1:5600` | A separate JPEG fragment stream |

`SimulatorClient.open` binds exclusively. `poll` receives both streams; the first
MAVLink heartbeat pins the peer and target IDs. `write` sends commands to that
peer. MAVLink from other peers or system IDs is discarded. Camera packets are
assembled separately. Connecting does not launch or arm a simulator.

## observation to command

The controller path is explicit:

```python
state = client.read(timeout=...)
command = controller.update(state, gate_index, gates)
client.send(command)
```

`poll` keeps one cache of accepted MAVLink packets, annotated with host receipt
time. `read` builds the observation records directly from those packets.

`read` returns one new `HIGHRES_IMU` sample. `State.time` and `dt` are device
seconds; acceleration and gyro are three body components in m/s² and rad/s.
The latest motion, attitude, motor report, and frame each retain their own device
and host receipt timestamps. These samples are not synchronized. At controller
update, optional samples older than the session timeout become `None`.

| Wire observation | Controller value |
| --- | --- |
| `HIGHRES_IMU` | Body acceleration unchanged; all three gyro signs reversed to FRD |
| `LOCAL_POSITION_NED` | `Motion(position=Ned(x,y,z), velocity=Ned(vx,vy,vz))`, metres and m/s |
| `ATTITUDE` | Radians: roll unchanged, pitch and yaw reversed to FRD/NED |
| `ACTUATOR_OUTPUT_STATUS` | 32 original channel values and uint32 active mask; no RPM conversion |
| Camera `<IHHIIQ>` + JPEG bytes | Frame ID, chunk index/count, JPEG/payload lengths, device ns; completed readonly BGR image |

Race packets are `ENCAPSULATED_DATA` type 1, `<BQqqIq>`: type, boot ms, scheduled
start ms, finish ns, active gate index, last-gate value. The native gate index is
passed separately from `State`. Type 2 carries track fragments after
`DATA_TRANSMISSION_HANDSHAKE`; each gate is `<H9f>`: ID, NED base, wxyz orientation,
width, height. A complete track replaces the cached tuple. It is not republished
as a fresh observation on each controller cycle. See [track input](vehicle-api.md#track-input).

| Returned command | Active fields written to the executable | Native responsibility |
| --- | --- | --- |
| `PositionNed(n,e,d)` | `SET_POSITION_TARGET_LOCAL_NED`, frame 1, mask `3576`; position metres | Position and lower control loops |
| `VelocityNed(n,e,d)` | Same message/frame, mask `3527`; velocity m/s | Velocity and lower control loops |
| `BodyRates(r,p,y,thrust)` | `SET_ATTITUDE_TARGET`, mask `144`; negated physical rad/s and thrust 0..1 | Rate feedback and motor output |

A mask bit of 1 ignores its field. The body-rate mask ignores the quaternion
(bit 7) and selects the vendor's rad/s extension (bit 4). Position and velocity
ignore acceleration, yaw, and yaw rate. Ignored vectors are zero-filled; a zero
in an active vector remains a request. The command timestamp is host monotonic
milliseconds since the connection opened, wrapped to uint32. It is not IMU or
race time. `COMMAND_LONG` arm/disarm is separate from the setpoint message.

## the remaining wire interface

`client.telemetry` retains every accepted MAVLink packet by message name, including
`HEARTBEAT`, `TIMESYNC`, `ODOMETRY`, `COMMAND_ACK`, and `COLLISION`. Their original
fields remain available even when they are not part of `State`. A packet appears
only if the executable emits it; the cache does not request extra sensors.

`client.mav` is the same pymavlink encoder/parser used by the adapter.
`client.target_ids` is the `(system, component)` tuple from the first heartbeat.
After `connect`, the vendor time-sync request can be sent directly:

```python
import time

client.mav.timesync_send(time.time_ns(), 0)
reply = client.telemetry.get("TIMESYNC")  # poll or read to receive a reply
```

A native VQ1 build-3391 check received replies to all 97 requests during a complete
six-gate run. Each reply echoed the client request timestamp in `ts1` and returned
the simulator timestamp in `tc1`. The adapter makes no clock adjustment.

The [vendor controller](reference/PyAIPilotExample-v4/controller.py) also writes
`SET_ACTUATOR_CONTROL_TARGET` with eight controls, group 0, and the target IDs,
and `COMMAND_LONG` with command 31000 to reset the simulator. Both are available
through `client.mav`, as are the quaternion, acceleration/force, and heading
fields of the two flight messages. These are raw wire requests: `send` alone
applies the documented frame conversions for the three flight command types.

The UDP fixture checks outgoing time-sync, actuator, and reset requests, incoming
time-sync replies, and retained odometry, acknowledgement, and collision fields.
It establishes the wire interface; it does not establish native flight behavior
for the broader control forms. The six-gate native regressions exercise position
and body-rate commands; the UDP fixture also checks velocity commands.

## inside the executable

The native rate path was inspected in VQ1 executable SHA-256
`d5bec020a98a0def0bf5b124b57d38189e78fbb173a5ec81697aad25c0efbec9`:

```text
SET_ATTITUDE_TARGET receiver 0x14104b4f0
  -> cached mode/rates/thrust, published by 0x14104e510
  -> input getter 0x14104ac50
  -> command consumer 0x141394cd0
  -> control tick 0x14138f900
  -> native inner-controller dispatch and retained outputs
```

The receiver selects rate or attitude mode from the mask and retains ignored
fields. Bit 4 selects the vendor's physical-rate interpretation. In that path,
`0x1413901e0` maps rates back through the configured rate curve before the inner
controller; the packet is not sent directly to motors. The control tick passes
commands and angular feedback to its inner controller. Python `r1_body_rates` closes position and attitude;
it does not replace the executable's inner control or rigid-body model.

The specification describes thrust, drag, gravity, collision, and a 120 Hz
physics update. The executable owns these dynamics and the telemetry/camera
producers. The exact motor-output-to-force calls, PhysX integration step, sensor
sampling/filtering, and their relative tick order have not been fully traced.
The addresses above establish the command handoff, not a complete physics model.
[Measured body-rate responses](body-rates.md) and complete native flights provide
behavioral checks across that boundary.

## session clock and shutdown

The session has three phases: `startup` waits for native GO; `control` reads,
updates, and sends; `finish` receives after stopping actuation on an IMU outage.
One deadline bounds startup and the later five-second finish window. Control
runs only in `control`; recovering IMU during `finish` does not rearm the vehicle.
A controller can wait for required observations before its first command.
The first valid command is checked before arming. Native finish terminates the
session even without another IMU sample. See [race lifecycle](race-lifecycle.md).

Heartbeat is 2 Hz and control defaults to 50 Hz; the PDF requires commands below
100 Hz. These are host schedules, separate from the specified physics rate and
30 Hz camera stream. Cleanup sends zero body rates/thrust and disarm when armed,
stops the owned Wine process, then closes the sockets. Attached runs own the
connection and commands but leave process lifetime with the external launcher.

The cleanup order is an explicit `try/finally` tree. A failed neutral command still
attempts disarm; a failed Wine shutdown still attempts process cleanup; the client
sockets close last. Cleanup failures are reported instead of being suppressed.
The one `contextmanager` shares Wine lifetime handling with the launcher-only run;
attached runs yield no owned process.
