# Race lifecycle

Race timing belongs to the simulator session, not the vehicle's physics.

The existing startup machinery loads the track, assigns a starting-grid slot,
confirms readiness, and enters the native race flow. Neither receiving IMU data
nor the launcher's `physics_ready` log is permission to command flight.

`ENCAPSULATED_DATA` type 1 carries `<BQqqIq>`: type, simulator boot milliseconds,
scheduled race-start boot milliseconds, finish value, active gate index, and
last-gate value. Negative start means no scheduled start. A nonnegative start
can still be in the future.

The shared runner follows this sequence:

| Signal | Action |
| --- | --- |
| No race packet, or negative start | Wait; heartbeat only. |
| Boot time is below scheduled start | Countdown; heartbeat only. |
| Fresh packet with boot time at or beyond start | Run the controller, then arm and send its first valid command. |
| Race packets pause after GO, but required sensors remain fresh | Keep the confirmed running phase and last reported gate. |
| Active gate index increases | Target the next gate. |
| Nonnegative finish value | Zero thrust, disarm, stop the owned simulator. |
| IMU stops for one second during the race | Zero thrust and disarm, then receive for up to five more seconds for native finish. |
| No finish within that receive window | Report telemetry failure and stop the owned simulator. |
| Race reset, process exit without finish, controller error, or Ctrl+C | Stop immediately and disarm if armed. |

GO uses the two timestamps in the same race packet. The runner does not
extrapolate countdown completion from host time or IMU time. The first GO must
come from a packet received within the last second. Before GO, the startup
deadline bounds waiting without arming, including a paused IMU stream.

After GO, race state is latched until native finish or reset. It is not a sensor
sample and does not expire because a periodic update was lost. Gate progress
remains the last reported index; neither elapsed time nor passing a target
position advances it. The heartbeat is 2 Hz; control defaults to 50 Hz.

VQ2 R2 restarts its telemetry clock during startup. A backwards boot timestamp
in a pre-GO race packet resets the client's IMU timestamp filter. The next
sample starts with `dt=0`. Backwards IMU timestamps alone are still rejected;
a race reset after GO stops the run.

Race completion does not require another IMU sample. The runner checks the
latest race packet during bounded IMU waits and keeps sending heartbeats.
An IMU outage first stops actuation, not the receive path. The runner then allows
five seconds for a delayed finish packet from the same race. It does not replay
commands or rearm if IMU recovers during that window. Without native finish the
outage remains an error, never a completed race. A queued finish is checked
before reporting process exit.

Finish is a terminal event and does not expire with receipt age. The start
timestamp and monotonic boot/gate checks still reject a reset or finish from a
different race. No gate count, telemetry silence, or process exit implies finish.

## Native evidence

Read-only Ghidra inspection of VQ2 build 3391, executable SHA-256
`68dfd80d5c9057ec92785baad61194bf5d178ddde8d6df66a6add4da5d83332b`:

- `0x14104cb20` writes the scheduled start at module offset `+0x500` as the
  current boot clock plus a countdown duration. Its caller is `0x1413c66f0`.
- `0x14104c730` serializes current boot time and scheduled start separately into
  the type-1 payload. The race worker sends this stream on a nominal 250 ms cadence.
- `0x1413e70a0` increments the passed-gate index and calls `0x14104caa0` to publish
  it. A position target being reached is not the gate-pass signal.
- `0x14137e850` calls `0x14104cb00` to publish the finish value.
- `0x14104cb00` stores finish at module offset `+0x508`; the periodic builder
  reads it without clearing it. Start (`+0x500`), gate (`+0x510`), and last-gate
  time (`+0x518`) are also retained fields, not one-second activity leases.
- The VQ1 builder at `0x14104c730` reads the same four retained fields. Finish is
  written by `0x14104d040`. No native rule turns a gap in race updates into finish.

The bundled [receiver](reference/PyAIPilotExample-v4/mavlink_rx.py) supplies the
wire layout. The [specification](VQ1-Technical-Specification-00.02.pdf), page 8,
specifies the heartbeat minimum. No binary or Lua changes are needed for this fix.

Unit tests cover pending start, future start, fresh GO, race packet gaps, finish,
reset, process exit, and bounded shutdown after IMU loss. The loopback child-process
tests check that no arm or position command is sent before GO, then exercise six
gate transitions and shutdown, including a three-second race packet gap.
Those tests validate sequencing and transport, not Unreal flight dynamics.
The finish tests cover continued IMU, a finish packet with no final IMU sample,
finish delayed until after disarm, and telemetry loss without any finish signal.
