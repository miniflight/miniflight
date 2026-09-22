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
| Active gate index increases | Target the next gate. |
| Nonnegative finish value | Zero thrust, disarm, stop the owned simulator. |
| Race reset, stale telemetry, controller error, or Ctrl+C | Zero thrust and disarm if armed, then stop the owned simulator. |

GO uses the two timestamps in the same race packet. The runner does not
extrapolate countdown completion from host time or IMU time. Race packets have
their own receipt timestamp, so camera/IMU/track traffic cannot keep stale race
state alive. The heartbeat is 2 Hz; control defaults to 50 Hz.

VQ2 R2 restarts its telemetry clock during startup. A backwards boot timestamp
in a pre-GO race packet resets the client's IMU timestamp filter. The next
sample starts with `dt=0`. Backwards IMU timestamps alone are still rejected;
a race reset after GO stops the run.

Race completion does not require another IMU sample. The runner checks the
latest race packet during bounded IMU waits and keeps sending heartbeats.
Without a finish packet, a one-second IMU gap is still an error; it is never
treated as a completed race.

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

The bundled [receiver](reference/PyAIPilotExample-v4/mavlink_rx.py) supplies the
wire layout. The [specification](VQ1-Technical-Specification-00.02.pdf), page 8,
specifies the heartbeat minimum. No binary or Lua changes are needed for this fix.

Unit tests cover pending start, future start, exact GO boundary, finish, reset,
and stale packets. The loopback child-process test checks that no arm or position
command is sent before GO, then exercises six gate transitions and shutdown.
Those tests validate sequencing and transport, not Unreal flight dynamics.
The finish tests cover both continued IMU and a finish packet with no final
IMU sample, plus telemetry loss without a finish signal.
