# AI-GP simulators

## Setup

Requires macOS and `/Applications/Game Porting Toolkit.app`.

```sh
brew install git-lfs uv
git lfs install
git clone https://github.com/miniflight/miniflight.git
cd miniflight
git lfs pull
```

## Run

VQ1:

```sh
./target/aigp/run vq1.r1 --controller r1_gates
```

VQ2 R1:

```sh
./target/aigp/run vq2.r1 --controller zero
```

VQ2 R2:

```sh
./target/aigp/run vq2.r2 --controller zero
```

One command starts both simulator and controller. Ctrl+C stops both.
`r1_gates` is the VQ1 six-gate position-control baseline. `zero` sends zero thrust.
Replace the controller name with your module in `controllers/`.
Omit `--controller` to run only the simulator. Run one simulator at a time.

`client.py` handles UDP. `_runtime/` installs and runs the simulator and controller.
`archives/` holds one archive per simulator and `docs/` holds its reference material.
`.runtime/` holds local installations and caches and is ignored by Git.

[Controllers](../../controllers/README.md) · [Specification](docs/VQ1-Technical-Specification-00.02.pdf) · [Bundled Python example](docs/reference/PyAIPilotExample-v4)
