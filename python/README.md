# Python client

This is how your own robot-control code talks to the simulator: connect, read sensors, send
commands. Everything goes over gRPC (`simulator/proto/simulator.proto`) on `localhost:<grpc_port>`
(`network.grpc_port` in `simulator/configs/project.json`, default `50051`) — the simulator must
already be running (`./run_simulator.sh`).

You do not need to touch gRPC or the generated protobuf code directly. `robot_hal.py` wraps all of
it behind one class, `SimRobotHAL`, with plain method calls and Python-native types (floats,
tuples, numpy arrays). This is the intended integration point for your control code — see
"Why a HAL, not raw gRPC" below for why it's shaped this way.

## Using this from your own project (recommended)

Clone this repo as a subfolder of your own robot-control project, next to your own code:

```
my_robot_project/
├── robocup_simulator/        <- git clone https://github.com/c1paa/robocup_simulator.git
├── src/                      <- your own control code
└── files/                    <- anything else of yours (data, configs, ...)
```

```bash
cd my_robot_project
git clone https://github.com/c1paa/robocup_simulator.git

python3 -m venv .venv && source .venv/bin/activate
pip install -e ./robocup_simulator/python        # registers `robot_hal` as an importable module
bash robocup_simulator/simulator/setup.sh        # one-time: builds the C++ simulator itself
```

Then, from any file under `src/` (or anywhere else in your project — no relative paths, no
`sys.path` editing):

```python
from robot_hal import SimRobotHAL

hal = SimRobotHAL()
hal.connect()
hal.send_velocity(0.5, 0.0, 0.0)
```

`pip install -e` is an *editable* install: it doesn't copy `robot_hal.py` anywhere, it just makes
your venv import it straight from `robocup_simulator/python/`, so `git -C robocup_simulator pull`
picks up updates with no re-install needed. The generated gRPC stubs
(`robocup_simulator/python/generated/`) are also handled for you — `robot_hal.py` generates them
itself the first time it's imported (and regenerates them if `simulator.proto` ever changes and you
pull), so there's no separate `generate_proto.sh` step to remember either. You still need the
simulator itself running before your code connects — build it once with `setup.sh` above, then
`./robocup_simulator/run_simulator.sh` each time you want to run it.

## Quick start (working inside this repo itself)

If you're editing this repo directly rather than depending on it from elsewhere — e.g. hacking on
`viewer.py`/`lidar_viewer.py`, or adding a new sensor to `robot_hal.py` — a plain
`pip install -r requirements.txt` (or just running scripts from inside `python/`) works too, no
editable install needed:

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -r python/requirements.txt
```

```python
from robot_hal import SimRobotHAL   # run from inside python/, or sys.path.insert it first

hal = SimRobotHAL()   # defaults: host="localhost", port=50051, robot_id=0
hal.connect()          # starts the background sensor/command threads

hal.send_velocity(0.5, 0.0, 0.0)   # drive forward at 0.5 m/s (body-frame)
hal.dribble(1.0)                    # spin the dribbler in the capture direction
print(hal.get_pose())               # (x, y, z, yaw) ground truth

hal.kick(1.0)                       # fire the kicker at full requested power

hal.close()
```

Either way, `import robot_hal` (or `from robot_hal import SimRobotHAL`) is the only import your
control code needs from this project — no manual channel/stub setup, no protobuf message
construction.

## API reference

### Lifecycle

| Method | Description |
|---|---|
| `SimRobotHAL(host="localhost", port=50051, robot_id=0)` | Construct the client. `robot_id` lets multiple robots share one simulator once multi-robot support lands (see `ROADMAP.md` item 6) — today there is exactly one robot, always `robot_id=0`. |
| `connect()` | Starts two background daemon threads: one reading the `SensorStream`, one draining a command queue into `SendCommand`. Idempotent — safe to call once, ignored on repeat calls. |
| `close()` | Stops both background threads and closes the gRPC channel. Call this when your control loop exits. |

### Sensors (all read the single most-recently-received `SensorData` message; each returns `None` until the first message arrives after `connect()`)

| Method | Returns | Notes |
|---|---|---|
| `get_camera_frame()` | `(H, W, 3)` uint8 `numpy` array, or `None` | The robot's own mirror-camera image (real mirror-distorted, not undistorted) — what an actual onboard camera would see. Feed straight into OpenCV. |
| `get_pose()` | `(x, y, z, yaw)` | Ground truth (meters, radians) — position and yaw. Use for debugging/scoring, not as a stand-in for what a real robot would know (a real robot only has `get_odometry()` + `get_camera_frame()`). |
| `get_odometry()` | `(x, z, yaw)` | Dead-reckoning estimate integrated from commanded wheel speeds, deliberately drifts away from `get_pose()` under wheel slip — this is what a real robot's encoders would report. This is the one to use if you're writing control code meant to also run on real hardware later. |
| `get_ball_position()` | `(x, y, z)` | Ball ground truth (meters). No onboard ball-detection sensor exists (that's what `get_camera_frame()` + your own vision code is for) — this is for debugging/scoring only. |
| `get_dribbler_rpm()` | float | Dribbler roller's actual (lagged, load-sagged) speed in RPM, signed like `dribble_speed` (positive = capture direction). |
| `get_capacitor_charge()` | float, `0.0`-`1.0` | Kicker capacitor charge. Poll this before `kick()` if you want to know whether a kick will actually be at full power — see `kick()` below. |
| `get_lidar_scan()` | `(N, 3)` float32 `numpy` array `[angle, distance, intensity]`, or `None` | Latest *completed* 360° sweep (paced by its own scan rate, independent of the sensor stream's frame rate). `angle` is radians in the robot's body frame, `0` = forward (`+X`), positive = left (same sense as `omega`/yaw). `distance` in meters, `intensity` a synthetic `0.0`-`1.0` confidence value. |

### Commands

| Method | Description |
|---|---|
| `send_velocity(vx, vy, omega)` | Body-frame drive command: `vx` forward (m/s), `vy` lateral (m/s), `omega` yaw rate (rad/s). Clamped simulator-side to the configured motor limits (`/robot/motor/max_linear_speed`/`max_angular_speed` in `robot.json`). Persistent — keeps applying every tick until you call it again (e.g. with all zeros to stop). |
| `dribble(speed)` | Dribbler roller speed, `-1.0` to `1.0`. Positive = capture direction (pulls a ball in and holds it), negative = eject. Persistent, same as `send_velocity`. |
| `kick(power)` | Requests a kick at `power` in `[0.0, 1.0]`. **One-shot**, unlike the two above — it rides along on a single command and is not re-sent when you next call `send_velocity()`/`dribble()`, so it won't re-fire on its own. The impulse actually delivered is `min(power, capacitor_charge) * max_power` — a kick requested before the capacitor has recharged (`/robot/kicker/charge_time` after the last kick) will be measurably weaker; poll `get_capacitor_charge()` first if that matters to your control logic. Only does anything if the ball is within `/robot/kicker/range` of the kick point. |

`send_velocity`/`dribble`/`kick` all only take effect if the ball/robot is actually in the right
physical state (e.g. `kick()` needs the ball within range) — none of them raise on a no-op call,
they just don't do anything that tick.

## Why a HAL, not raw gRPC

`SimRobotHAL` is deliberately the same shape a future `HardwareRobotHAL` (talking to real
motors/camera/lidar over whatever real transport, no simulator involved) would have — same method
names, same units, same body-frame convention. Write your control logic against `SimRobotHAL`'s
interface and it should need minimal changes to run on real hardware later; that's the whole point
of routing everything through this one class instead of importing `generated/simulator_pb2*` and
gRPC stubs directly in your own code.

## Robot frame convention

`vx`/`vy` are in the robot's own body frame, not world/field coordinates: `vx` is always "forward
from the robot's current heading," `vy` is always "left/right relative to the robot," regardless of
`yaw`. `omega`/yaw follow a fixed rotation sense: positive `omega` turns the robot's `+X` (forward)
toward `-Z` in world coordinates. If you're building a world-frame controller (e.g. "drive to field
position X,Z"), rotate your world-frame velocity into body frame using the current `yaw` from
`get_pose()`/`get_odometry()` before calling `send_velocity()`.

## Other files here

- `viewer.py` — minimal OpenCV camera demo. Predates `robot_hal.py` and talks raw gRPC directly
  (its own `simulator_pb2`/`grpc` calls, not `SimRobotHAL`) — useful as a bare-minimum reference for
  what the HAL is doing under the hood, but write new code against `SimRobotHAL.get_camera_frame()`
  instead of copying this pattern.
- `lidar_viewer.py` — live 2D lidar plot, *does* go through `robot_hal.SimRobotHAL.get_lidar_scan()`
  — the more representative example to copy from.
- `generate_proto.sh` — regenerates `generated/` from `simulator/proto/simulator.proto` on demand.
  `robot_hal.py` already does this automatically on import when needed (see above), so you'll
  rarely run this by hand — it's there mainly for explicitly forcing a regeneration.
- `pyproject.toml` — makes `python/` installable with `pip install -e .` (see "Using this from your
  own project" above). `robot_hal` is a flat module, not a package, so nothing else here changes.
