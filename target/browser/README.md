# Exact AI-GP browser target

This target keeps the AI-GP executable and its native simulation behavior. The
browser runtime is still under development. A complete native race, rendered
scene, and native camera stream have not been demonstrated in the browser.

The experimental page runs a startup check with the original engine and
`-nullrhi`. It keeps the gate controller disabled until native GO, fresh IMU,
pose, and track packets arrive. To use the retained runtime in this checkout:

```sh
python3 -B target/browser/prepare.py
python3 -B target/browser/serve.py
```

Open the printed browser URL, select the prepared AI-GP folder, and start the
original simulator. The Python process serves static runtime files; the game,
file mount, MAVLink, and controller execute inside the tab. Preparation links
the nine existing runtime assets without copying the game. Those generated
runtime files are not included in Git.

The adapters here move asset access, MAVLink, camera assembly, and controller I/O
into the tab. They contain no replacement physics.

`assets.mjs` exposes the original browser-selected files as a virtual STORE ZIP64.
Only ZIP metadata and requested slices are assembled. The 4.91 GB package is not
copied into another archive. `verify-assets.mjs` checks every file's CRC32 and the
executable's SHA-256 in a worker before mounting it. The manifest describes the
prepared VQ1 package; it includes the existing UE4SS and DirectVQ1 configuration.

The executable SHA-256 is
`d5bec020a98a0def0bf5b124b57d38189e78fbb173a5ec81697aad25c0efbec9`.

The runtime's lazy ZIP reader uses `fetch` on the browser main thread. After file
verification, install the file-backed fetch adapter before starting the runtime:

```js
import {GameArchive, selectedFiles} from './assets.mjs';

const files = selectedFiles(folderInput.files); // input type=file webkitdirectory
const archive = new GameArchive(manifest, files);
const url = new URL('./game.zip', location.href).href;
const restoreFetch = archive.installFetch(url);
const mount = 'bw64url:' + archive.size + ';' + url + '|' + archive.size;
// Pass mount as a -zip argument to the browser runtime.
```

`transport.mjs` carries the runtime's loopback UDP messages entirely in the tab.
Install it before loading the emulator so its MAVLink and camera sockets use the
browser client. Other WebSocket destinations retain their normal behavior.

```js
import {browserDatagrams} from './transport.mjs';
import {SimulatorClient} from './client.mjs';

let sim;
const transport = browserDatagrams((bytes, port, peer) => sim.receive(bytes, port, peer));
sim = new SimulatorClient((bytes, peer) => transport.send(bytes, 14550, peer));
```

`sim.telemetry` retains decoded fields, timestamps, source IDs, and raw packets for
all 205 common MAVLink messages in the pinned pymavlink 2.4.49 schema. `sim.read()`
returns fresh IMU observations with independent pose, attitude, motor, and camera
freshness. `sim.send()` accepts BodyRates, PositionNed, and VelocityNed commands
using the existing AI-GP wire conventions. Native heartbeat, GO, gate progression,
and finish packets must determine readiness and race state.

Run the adapter checks from the repository root:

```sh
node test/browser_io.mjs
node test/browser_assets.mjs
python3 -B test/browser_assets_zip.py
```

The I/O fixture checks 12 incoming packets and five outgoing packets against
pymavlink. Python's independent ZIP reader checks the browser writer's 64-bit
sizes and offsets without allocating a large payload. An actual Chrome check
verified all 163 prepared files and compared 491 ZIP ranges against the existing
package, including offsets above 4 GB. These are adapter checks, not evidence of
a completed native simulation tick.

The new frontend also passed a native read-only PAK probe: its interpreter opened
the browser-backed ZIP mount, reported the exact 4,574,387,295-byte PAK size, and
matched original bytes at offsets 0, 2^32 + 17, and end - 64. It exited with status
0, with no game-file HTTP requests. This check used the retained ELF probe rather
than starting a second game instance.
