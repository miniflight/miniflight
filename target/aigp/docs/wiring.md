# simulator wiring

The entrypoint is [simulator.py](../simulator.py). [client.py](../client.py) owns
packet receipt and encoding; [native.py](../native.py) owns the executable.
The [VQ1 specification](VQ1-Technical-Specification-00.02.pdf),
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

Build/course selection is separate from official event selection. Both Lua
launchers call `ServerForceStartRaceWithoutRaceVerification` directly; neither
selects the native Training or Qualification event block. The runner therefore
labels owned runs as direct arena races. Attached runs have an unverified external
event context. The race packet contains no event ID or training/qualification
field, so its GO and finish cannot establish that context. VQ2 section 9.2
requires selecting the corresponding event block in the native interface.

## the two UDP connections

| Stream | Native socket | Controller socket | Data |
| --- | --- | --- | --- |
| MAVLink 2 | The heartbeat's sender, normally `127.0.0.1:14560` | `127.0.0.1:14550` | Commands, IMU, optional pose/motors, race, track |
| Camera | Normally `127.0.0.1:5601` | `127.0.0.1:5600` | A separate JPEG fragment stream |

Each row is a direct UDP stream between two programs. The controller socket is
where the pilot receives packets; it is not a proxy or a second simulator API.
MAVLink commands and heartbeats go back from that socket to the native sender.
The camera is a separate one-way stream. These addresses are transport details
under one CLI command.

`SimulatorClient.open` binds exclusively. `poll` receives both streams; the first
MAVLink heartbeat pins the peer and target IDs. `write` sends commands to that
peer. MAVLink from other peers or system IDs is discarded. Camera packets are
assembled separately. Connecting does not launch or arm a simulator.

A passive VQ1 build-3391 probe sent heartbeats and time-sync requests from an
ephemeral socket to native UDP 14560. That initiating socket received no packets.
The fixed listener on 14550 received 67 heartbeats, 247 IMU samples, and 16
time-sync messages, as well as race, course and privileged state packets. With
these native defaults, merely connecting an arbitrary socket to 14560 does not
redirect the telemetry stream; the pilot must listen on the native destination.

## how the vendor starts a controller

Both original build-3391 packages contain a native simulator zip and a separate
`PyAIPilotExample-v4.zip`. Their README tells the user to launch `FlightSim.exe`,
log in inside the simulator, and run `python main.py` separately. All seven SDK
files in both Downloads packages match [our vendor reference](reference/PyAIPilotExample-v4/)
byte for byte.

The SDK [setup.py](reference/PyAIPilotExample-v4/setup.py) imports `Controller`
from `controller.py`, opens `udpin:127.0.0.1:14550`, waits for the native heartbeat,
then creates the controller in the Python process. [main.py](reference/PyAIPilotExample-v4/main.py)
arms and calls its `update` loop. The supplied integration does not load that
Python class into Unreal. Our CLI likewise imports the chosen controller in the
pilot process and exchanges native MAVLink packets.

Controller startup and native event startup are separate jobs. The SDK sample
arms after the heartbeat; our direct arena waits for GO before arming. Automating
the official menu flow must preserve its actual readiness and race-verification
sequence, rather than treating the direct arena sequence as the official one.
The packages document no command-line option for selecting an official event.
The intended CLI work is to drive that native event path and start the external
pilot together.

## observation to command

The controller path is explicit:

```python
state = client.read(timeout=...)
command = controller.update(state, gate_index, gates)
client.send(command)
```

`poll` keeps one cache of accepted MAVLink packets, annotated with host receipt
time. `read` builds the observation records directly from those packets.

`read` returns the newest unread `HIGHRES_IMU` sample; intermediate IMU packets
may be skipped. It does not advance native physics. `State.time` and `dt` are device
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

The receive path makes these different arrivals explicit:

```text
DATA_TRANSMISSION_HANDSHAKE       -> _track.start: announce ID, size, fragment count
ENCAPSULATED_DATA, data[0] == 2    -> _track.receive: assemble, validate, publish gates
ENCAPSULATED_DATA, data[0] == 1    -> race_status: native start, finish and gate progress
HIGHRES_IMU / optional telemetry  -> telemetry[message name]: latest independent packet
Camera UDP fragments             -> _camera.receive: publish only a complete JPEG
```

An announcement or incomplete transfer leaves the previous gates available.
Only a complete usable transfer changes `client.gates` and
`client.gates_received_at` (host monotonic seconds). Both properties inspect the
cache without receiving packets. The gates do not expire when an IMU sample does.
`Gate.origin` retains the published NED base; `Gate.orientation` is its normalized
wxyz quaternion. `Gate.center` is derived from those values and the published height.

| Input | Arrival and availability at `update` |
| --- | --- |
| IMU | Required new sample on each call; device `dt` spans the returned samples. |
| Motion and attitude | Latest separate packet; reused across calls, or `None` if missing, nonfinite or older than the session timeout. |
| Motors | Latest reported channels and active mask; reused, or `None` if missing or stale. No RPM estimate. |
| Camera | Latest complete decoded JPEG; reused, or `None` if missing or stale. An incomplete image does not replace it. |
| Gate geometry | Cached complete course, or `None`; no per-call freshness rule. |
| Active gate index | Last native race packet; retained through packet gaps after GO. |
| Command | One return value per update; checked, then armed/sent; no native-step acknowledgement. |

A passive native VQ1 R1 check on 2026-10-07 opened the receiver before launch and
sent only 2 Hz GCS heartbeats. It used build 3391, executable SHA-256
`d5bec020a98a0def0bf5b124b57d38189e78fbb173a5ec81697aad25c0efbec9`.
All gate packets arrived between 9.468855 and 9.478258 host seconds after opening:
two transfer IDs (0 and 1), each announcing 230 bytes in one fragment, with each
announcement and fragment duplicated. Each decoded payload contained six gates.
Native GO was observed at 13.455426 seconds. No further gate packets arrived during
40 seconds after GO. This run establishes startup delivery, not a guaranteed
periodic publication rate. Starting the receiver late can miss the course.

The same run received privileged `LOCAL_POSITION_NED`, `ATTITUDE`, and `ODOMETRY`
as separate packets, beginning around 9.75 seconds. Motion and attitude become
optional `State` fields; odometry remains raw in `client.telemetry["ODOMETRY"]`,
including its quaternion, frame IDs and covariance fields. No synthetic pose is
filled in when a stream is absent. VQ2 [section 9.3](VQ2-Technical-Specification-00.03.pdf)
blocks these three privileged messages and `GATE_INFO`; VQ1 availability must not
be assumed for VQ2.

After this refactor, the native `r1_gates` regression completed all six gates,
checked a 640×360 camera image, and replayed all 1,859 controller updates exactly.
The UDP regression separately checks announcement without geometry, completion
without an IMU, repeated reads without a new transfer, replacement receipt time,
and rejection of redacted geometry without losing the previous course.

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

`client.telemetry` retains the latest accepted MAVLink packet per message name, including
`HEARTBEAT`, `TIMESYNC`, `ODOMETRY`, `COMMAND_ACK`, and `COLLISION`. Their original
fields remain available even when they are not part of `State`. A packet appears
only if the executable emits it; the cache does not request extra sensors.
It is not event history: a later `COLLISION` or `COMMAND_ACK` replaces the previous
one. Record arrivals through `_receive`, as the optional recording client does,
when every event matters.

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
`startup_deadline` bounds GO and required inputs before the first command;
`finish_deadline` starts only on IMU loss and bounds the five-second finish wait. Control
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
