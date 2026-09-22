# Agent / contributor guide

Read this before changing code — human or AI. It exists so different sessions (and different
people) don't drift into inconsistent conventions or quietly re-break things that were already
fixed once.

## Before you start

- Read [`ROADMAP.md`](ROADMAP.md) to see what's an intentional stub vs. a bug. A lot of things
  that look unfinished (the gRPC server, the camera image, the missing ball) are *known* and
  tracked there, not something to silently "fix" as a side effect of an unrelated task.
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
simulator/shaders/   reserved, currently unused (shaders are inline in renderer.cpp)
simulator/lib/       reserved, currently unused
```

Class responsibilities (see their headers for the exact interface):

- `App` — window/GL setup, main loop, viewer camera (orbit/pan/zoom), owns all subsystems.
- `Renderer` — thin immediate-mode-style OpenGL wrapper (grid/line/box/sphere/cylinder/cone).
- `Physics` — Bullet world lifecycle.
- `Field` — field geometry and rendering, driven entirely by config.
- `Robot` — single robot's kinematics, geometry, and rendering.
- `Camera` — the robot's own (mirror) camera, separate from the viewer camera in `App`.
- `GrpcServer` — external control/sensor interface (currently a stub, see `ROADMAP.md`).
- `Config` — JSON config singleton, dot-path lookup (`cfg.getFloat("/robot/diameter", ...)`).

## Conventions to keep consistent

- **Units**: config JSON is always millimetres; convert to metres exactly once, at load time,
  in the class that owns the field (see `static const float MM = 1000.0f;` pattern in
  `robot.cpp`, `camera.cpp`, `physics.cpp`). Never introduce a second unit convention or do the
  conversion at the call site.
- **Config access**: always go through `Config::instance().getFloat/getInt/getString(path,
  default)`. Every value needs a sane default — configs are optional, not required (see
  `App::init`, which only warns and falls back if the JSON files are missing).
- **Naming**: member variables use `m_camelCase`; classes are `PascalCase`; methods are
  `camelCase`. Match the surrounding file.
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
