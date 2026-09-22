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
./target/aigp/run vq1.r1
```

VQ2 R1:

```sh
./target/aigp/run vq2.r1
```

VQ2 R2:

```sh
./target/aigp/run vq2.r2
```

Run one at a time. Ctrl+C stops the simulator.
