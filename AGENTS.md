# Agent / contributor guide

Read this before changing code — human or AI. It exists so different sessions (and different
people) don't drift into inconsistent conventions or quietly re-break things that were already
fixed once.

## Before you start

- Read [`ROADMAP.md`](ROADMAP.md) to see what's an intentional stub vs. a bug. Things that look
  unfinished (currently: the missing ball, robot not hooked into Bullet physics, kicker/dribbler,
  single-robot-only) are *known* and tracked there, not something to silently "fix" as a side
  effect of an unrelated task.
- `git pull` (or `git pull --rebase`) before starting work in a fresh session — see
  [`CONTRIBUTING.md`](CONTRIBUTING.md) for the workflow.
- If you're about to build, run `bash simulator/setup.sh` once per machine, then
  `bash simulator/build.sh` / `./run_simulator.sh`. Never hand-edit `simulator/build/` — it's
  regenerated and git-ignored.

## Project layout (don't reorganize without discussing it)

```
simulator/src/       C++ source, one class per .h/.cpp pair
simulator/proto/     gRPC service definition — the external contract with client code
simulator/configs/   project.json, robot.json — runtime-tunable parameters
simulator/shaders/   reserved, currently unused (shaders are inline in renderer.cpp/camera.cpp)
simulator/lib/       reserved, currently unused
python/               Python/OpenCV demo client (`viewer.py`) + hardware-abstraction seed
                      (`robot_hal.py`, the `SimRobotHAL` gRPC wrapper)
```

Class responsibilities (see their headers for the exact interface):

- `App` — window/GL setup, main loop, viewer camera (orbit/pan/zoom), owns all subsystems.
- `Renderer` — thin immediate-mode-style OpenGL wrapper. `drawLine`/`drawBox`/`drawCylinder`/
  `drawCone` are wireframe (debug-viewer geometry); `drawMesh`/`drawSphere`/`drawSolidBox` are
  filled and lit (Lambertian, via `setLighting` — direction/ambient/diffuse are real uniforms set
  every draw call, not GLSL default-initializers, which aren't portable). `drawShadowBlob` is a
  cheap projected ground shadow (a flat disc multiplicatively darkening whatever's under it via
  `GL_DST_COLOR`/`GL_ZERO` blending, not a real shadow map) for the mirror-camera's own scene.
- `Physics` — Bullet world lifecycle.
- `Field` — field geometry and rendering, driven entirely by config; already used filled/lit
  meshes for the floor/walls/goals before `Renderer`'s lighting became configurable.
- `Robot` — single robot's dynamics (Bullet rigid body + 3-omni-wheel friction/slip model),
  geometry, and rendering; owns the `MirrorProfile` and the dead-reckoning odometry estimate. The
  chassis's *collision* cylinder is intentionally smaller than its rendered radius (by
  `/robot/dribbler/pocket_depth`) so a captured ball can sit partly recessed into the front of the
  robot instead of flush against it — see the comment in `Robot::init` and `Dribbler::init`'s
  `forward_offset` derivation. `renderCameraFrame` (used only by the mirror-camera capture pass,
  not `render`/`renderBody`) draws just the support pillars above the wheels instead of the full
  solid body — real hardware's underside is mostly open there.
- `Ball` — the (golf) ball: a plain Bullet `btSphereShape` dynamic rigid body with stock
  friction/restitution/rolling-friction/damping (no hand-rolled force model — unlike `Robot`),
  rendered in both the viewer and the robot's mirror-camera cubemap. See
  [`docs/tasks/ball-physics.md`](docs/tasks/ball-physics.md).
- `Dribbler` — the front capture roller: a *hand-rolled* friction/centering force model applied
  to the ball every frame (deliberately not a Bullet roller body — sustained cylinder-vs-sphere
  contact was solver-unstable), with motor lag, load-based speed sag, and a tapered-roller
  capture zone. Owned by `App` (needs both robot pose and ball state). See
  [`docs/tasks/dribbler-kicker.md`](docs/tasks/dribbler-kicker.md).
- `Kicker` — the solenoid kicker: an impulse (`applyImpulse` at the configured plunger-height
  offset) capped by a capacitor charge that is actually simulated (drains on fire, recharges
  over `charge_time`). Negative `height_offset` also tilts the impulse vector itself upward
  (`/robot/kicker/chip_max_angle`, scaled by how far below center the offset is) to produce a real
  chip — a pure position-offset impulse can't do this on its own, since `applyImpulse`'s linear
  velocity change is independent of the offset point (see the comment above
  `Kicker::requestKick`). Owned by `App`, same pattern as `Dribbler`. See
  [`docs/tasks/dribbler-kicker.md`](docs/tasks/dribbler-kicker.md).
- `MirrorProfile` — mirror shape (`cone`/`hyperbola`/`profile`) as a profile function
  `r = f(h)`; the single source of truth for mirror geometry, used both for the drawn mesh
  (`Robot`) and the optics (`Camera`). `type: "profile"` loads a real (or externally-measured)
  mirror from a CSV table (`theta_deg,r_mm,z_mm`, relative to the camera's own focal point)
  instead of a closed-form curve — see [`docs/tasks/mirror-camera-vision.md`](docs/tasks/mirror-camera-vision.md)
  for the coordinate convention and `simulator/configs/mirrors/` for example files.
- `Camera` — the robot's own (mirror) camera: cubemap capture + baked direction LUT →
  real mirror-distorted image, plus the in-window preview overlay. Separate from the viewer
  camera in `App`.
- `LidarSensor` — LD06-like 360° 2D scanning lidar: a Bullet raycast sweep (paced by its own
  `scan_frequency`, independent of the render/physics tick) with range/noise/dropout matched to
  the real sensor; publishes the latest completed scan over gRPC. Depends on the static
  field-boundary wall boxes added by `Physics`.
- `GrpcServer` / `SimulatorServiceImpl` — real gRPC server (`SensorStream`/`SendCommand`),
  bridged to the sim thread via the mutex-guarded `SharedState` (`shared_state.h`).
- `Config` — JSON config singleton, dot-path lookup (`cfg.getFloat("/robot/diameter", ...)`).

## Conventions to keep consistent

- **Units**: config JSON is always millimetres; convert to metres exactly once, at load time,
  in the class that owns the field (see `static const float MM = 1000.0f;` pattern in
  `robot.cpp`, `camera.cpp`, `physics.cpp`). Never introduce a second unit convention or do the
  conversion at the call site. The one exception is *mass*: `/physics/robot/mass` and
  `/physics/ball/mass` are in grams (converted to kg with the same `/ 1000` pattern, but a
  different physical quantity — see `Robot::init` / `Ball::init`).
- **Config access**: always go through `Config::instance().getFloat/getInt/getString(path,
  default)`. Every value needs a sane default — configs are optional, not required (see
  `App::init`, which only warns and falls back if the JSON files are missing).
- **Naming**: member variables use `m_camelCase`; classes are `PascalCase`; methods are
  `camelCase`. Match the surrounding file.
- **Robot local frame / yaw convention**: the robot's body frame is `+X` forward, `+Y` up,
  `+Z` lateral, and yaw rotates `+X` **toward `-Z`** for positive `omega`/increasing yaw — i.e.
  local→world is `worldX = cosYaw·localX + sinYaw·localZ`, `worldZ = -sinYaw·localX +
  cosYaw·localZ` (this is exactly what `glm::rotate(mat, yaw, (0,1,0))` produces, and what
  `Camera`'s composite shader uses — see `mirror-camera-vision.md`). Bullet's own reported
  angular-velocity Y component already matches this `omega` sign directly, no negation needed.
  An earlier version of this file (and of `docs/tasks/omni-wheel-dynamics.md`) had this
  backwards (`+X` toward `+Z`), which produced a robot that drove in the mirror image of the
  direction it was visually facing — fixed during review; if you're implementing something new
  against the yaw convention, verify empirically (drive-after-turning, compare against the
  rendered/mirror-camera orientation) rather than trusting a written sign description, this one
  included.
- **No hardcoded machine-specific paths.** Anything path-related goes through
  `SCRIPT_DIR`-style resolution in the shell scripts or `--config-dir` at runtime — never an
  absolute `/Users/...` path in source, config, or scripts. (This bit the project once already:
  a committed `simulator/build/` directory baked in one contributor's home directory and broke
  the build for everyone else. `simulator/build/` is git-ignored specifically to prevent that
  from happening again — don't remove it from `.gitignore` and don't force-add the directory.)
- **New third-party dependencies** go in `simulator/CMakeLists.txt` via `find_package` (prefer
  Homebrew-installable libraries) and get added to the `brew install` line in
  `simulator/setup.sh` so `setup.sh` remains the single command that gets a fresh machine
  working.

## Workflow

See [`CONTRIBUTING.md`](CONTRIBUTING.md) for branching, commit, and pull/push conventions.
