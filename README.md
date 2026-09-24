# RoboCup Vision Simulator

A single-robot 3D simulator for RoboCup Junior Soccer (Vision/Open league). It renders a field
and a robot with a catadioptric (mirror) omnidirectional camera, steps physics with Bullet, and
is meant to expose robot control + sensor data to external clients over gRPC.

See [`ROADMAP.md`](ROADMAP.md) for what's implemented vs. still a stub, and
[`AGENTS.md`](AGENTS.md) for conventions to follow when changing the code (human or AI).

## Requirements

- macOS with [Homebrew](https://brew.sh/) (Linux should work via the `find_package` calls in
  CMake, but is untested — see [`ROADMAP.md`](ROADMAP.md))
- CMake 3.20+, a C++17 compiler
- Dependencies: SDL2, OpenGL, Bullet Physics, gRPC, Protobuf (installed by `setup.sh`)

## Setup (first time, or after pulling on a new machine)

```bash
bash simulator/setup.sh
```

This installs missing Homebrew packages (`cmake sdl2 grpc protobuf bullet`), writes
`run_simulator.sh` in the repo root, and builds the project. Safe to re-run any time.

## Build only

```bash
bash simulator/build.sh
```

## Run

```bash
./run_simulator.sh
```

Builds automatically first if the binary is missing. Options:

```
--config-dir <path>   Path to configs directory (default: simulator/configs/)
--help, -h            Show help
```

### Viewer controls

- Right mouse drag — orbit
- Scroll — pan (hold Shift to orbit, Cmd to zoom)
- WASD — pan the look target
- Arrow keys — drive the robot directly (Up/Down = forward/back, Left/Right =
  turn), independent of gRPC `SendCommand` — handy for trying it out without a
  Python client
- Q / E — strafe the robot left / right
- Space (hold) — spin the dribbler at full capture speed; F — fire the kicker,
  both independent of gRPC the same way the drive keys are
- C — toggle the in-window mirror-camera preview overlay
- L — toggle the telemetry overlay (position/odometry, dribbler RPM, kicker
  capacitor charge, ball position relative to the robot)
- Esc — quit

## Python client

A minimal OpenCV client streams the robot's mirror-camera feed over gRPC:

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -r python/requirements.txt
bash python/generate_proto.sh          # once, regenerates python/generated/
```

Then, while the simulator is running:

```bash
python python/viewer.py                # optional: pass a different port
```

Press `q` or close the window to quit. `python/generated/` is generated code and
is git-ignored (same as `simulator/build/`).

To write your own robot-control code against the simulator (drive commands, camera, lidar,
dribbler, kicker, pose/odometry), see [`python/README.md`](python/README.md) — one `import
robot_hal` gets you the whole gRPC sensor/command interface behind plain Python method calls, no
manual gRPC/protobuf code needed.

## Project layout

```
simulator/
  src/        C++ source (App, Renderer, Physics, Field, Robot, Camera, MirrorProfile,
              GrpcServer, SimulatorServiceImpl, Config)
  proto/      simulator.proto — gRPC service definition (SensorStream / SendCommand)
  configs/    project.json (field/physics/viewer/network), robot.json (robot/mirror/camera)
  shaders/    reserved for GLSL files if shader source is ever pulled out of renderer.cpp/camera.cpp
  lib/        reserved for vendored libraries, currently unused
  setup.sh    one-time dependency install + build
  build.sh    incremental CMake build
python/       Python/OpenCV demo client for the gRPC camera stream (see "Python client" above)
run_simulator.sh   generated launcher (created by setup.sh)
```

`simulator/build/` is the CMake build directory — it's git-ignored, machine-specific, and must
never be committed (it embeds absolute paths from whoever built it).

## Config units

JSON config files (`configs/*.json`) use **millimetres** for all lengths, matching how robot
specs are normally given. Every value is converted to metres once, at load time, by the class
that reads it (search for `/ MM` in `src/*.cpp`). Don't mix units elsewhere — always read
config through `Config::getFloat/getInt/getString` and convert immediately.

## Current status

This is a work in progress. The viewer, field rendering, the real mirror-camera image, the gRPC
interface (`SensorStream`/`SendCommand`), the robot's 3-omni-wheel Bullet-physics drivetrain
(friction/slip, dead-reckoning odometry), the ball, dribbler/kicker, a lidar sensor, and field-wall
collision all work. Only one robot is supported (see `ROADMAP.md` item 6 for what multi-robot would
need). Full breakdown and priorities are in [`ROADMAP.md`](ROADMAP.md).
