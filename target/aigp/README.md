# VQ1 simulator

VQ1 1.0.3391 with UE4SS 3.0.1 and the existing DirectVQ1 startup mod.
Simulator binaries are stored in Git LFS as 1 GiB archive parts.
The launcher, Lua mod, and configuration are stored as ordinary Git files.

Use Git LFS and a current Python 3.11 or newer. Allow about 8 GiB for the
simulator archives, extraction, and temporary files, plus Git LFS storage.

Clone and download the files:

```sh
git lfs install
git clone https://github.com/miniflight/miniflight.git
cd miniflight
git lfs pull
```

On macOS, set up Python and restore the simulator:

```sh
python3 -m venv target/aigp/.runtime/client-venv
source target/aigp/.runtime/client-venv/bin/activate
python -m pip install -e '.[aigp,test]'
python target/aigp/extract_vq1.py
./target/aigp/run_vq1.sh
```

The macOS launcher requires `/Applications/Game Porting Toolkit.app` with
`Contents/Resources/wine/bin/wine64` and `wineserver` installed.

On Windows, use Command Prompt:

```bat
py -3 -m venv target\aigp\.runtime\client-venv
target\aigp\.runtime\client-venv\Scripts\activate.bat
python -m pip install -e ".[aigp,test]"
python target\aigp\extract_vq1.py
target\aigp\run_vq1.bat
```

The Windows launcher uses the native simulator executable. That launcher has
not yet been tested on Windows. There is no supported Linux launcher here.

Once the simulator reaches the race, open another terminal at the repository
root and activate the same Python environment. Then run:

```sh
python -m examples.vq1_connect
```

This prints the heartbeat, position, and velocity. If no heartbeat arrives,
the client waits. Position and velocity reads also wait if VQ1 is not sending
those messages. Use Ctrl+C to cancel. Only one client should use UDP port 14550.

To run the existing six-gate position-command demo:

```sh
python -m examples.aigp.thread_gates
```

`python -m examples.aigp.velocity_ned` runs the separate velocity experiment.
Its measured command rate is Python send throughput, not simulator physics rate.
Velocity-command behavior remains unverified in a live run.

On macOS, stop the client first, then press Ctrl+C in the simulator terminal.
The launcher stops Wine processes in VQ1's dedicated prefix. On Windows,
close the simulator window after stopping the client.

The archive excludes the separately tracked DirectVQ1 Lua script,
`Mods/mods.txt`, and `UE4SS-settings.ini`, so extraction preserves those files.
Wine prefixes, Python environments, logs, crash dumps, and duplicate downloads
are not included. VQ2 is not included.

`VQ1-Technical-Specification-00.02.pdf` is the official VQ1 specification,
document VADR-TS-002, issue 00.02, dated 2026-05-08. Page 5 names Virtual
Qualifier 1, page 8 lists attitude and linear-velocity telemetry, and page 11
describes Round One. It is not the VQ2 specification. VQ1 exposes privileged
state; do not assume those observations are available in VQ2.

This is a versioned reference, not a guarantee of every behavior in build 3391.
In particular, page 8 specifies a command rate below 100 Hz, while the bundled
Python example sets 250 Hz. The latter is not a verified simulator requirement.
Source: https://www.theaigrandprix.com/wp-content/uploads/2026/05/260508_Technical_Spec_0002.pdf
