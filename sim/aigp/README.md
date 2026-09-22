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
./sim/aigp/run vq1.r1 --controller r1_gates
```

VQ2 R1:

```sh
./sim/aigp/run vq2.r1 --controller zero
```

VQ2 R2:

```sh
./sim/aigp/run vq2.r2 --controller zero
```

One command starts both simulator and controller. Ctrl+C stops both.
`r1_gates` is the VQ1 six-gate position-control baseline. `zero` sends zero thrust.
Replace the controller name with your module in `controllers/`.
Omit `--controller` to run only the simulator. Run one simulator at a time.

[Controllers](../../controllers/README.md) · [Specification](../../docs/aigp/VQ1-Technical-Specification-00.02.pdf) · [Bundled Python example](../../docs/aigp/reference/PyAIPilotExample-v4)
