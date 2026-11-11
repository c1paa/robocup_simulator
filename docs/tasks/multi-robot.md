# Multi-robot support

Replaces the single hardcoded `App::m_robot` with an empty field that robots are added to "in
code": `App::addRobot(configPath, spawnX, spawnZ, spawnYawDeg)` constructs one robot (its own
`Robot`, `Camera`, `LidarSensor`, `Dribbler`, `Kicker`, `ImuSensor`, and its own slice of `Config`)
and registers it with `GrpcServer` under an auto-assigned `robot_id`. Calling it more than once
builds up a team (own robots and/or opponents) on one field, one gRPC port, multiplexed by
`robot_id` — exactly what `RobotCommand`/`SensorRequest.robot_id` were already carrying, unused,
since the very first version of the proto (see `ROADMAP.md` item 6's old wording). Matches the
user's request verbatim: "это симулятор по факту он пустой сначала, в коде мы можем создать
объект робота и подать ему в параметры путь до конфига и он добавлять на поле... можно
использовать 1 2 3 4 для переключения по разным роботам для текущего управления." A follow-up
request ("надо сделать чтобы в начале кода питона я мог просто отправить команды по типу создания
роботов с какимто конфигом а потому просто начинать работать") added a gRPC `AddRobot` RPC so the
same call is reachable from a running simulator, not just from C++ before `App::run()` -- see
"Dynamic robot creation" below.

## Why per-robot `Config`, not a bigger singleton

Every subsystem already took `Config& cfg` as an `init()` parameter rather than reading
`Config::instance()` directly (`Robot`, `Camera`, `LidarSensor`, `Dribbler`, `Kicker`, `ImuSensor`
— only `App::init` and `GrpcServer::start` touch the singleton, and both are project-/network-level
concerns, not per-robot). That meant giving each robot its own config was almost free: `Config`
gained a public default constructor (previously private, singleton-only) and
`Config::withRobotOverlay(path)`, which returns a **copy** of the calling config with `path`'s JSON
merged on top (same `merge_patch` semantics `loadRobot` already used). `App::addRobot` calls
`Config::instance().withRobotOverlay(robotConfigPath)` to build each robot's own config: a copy of
the shared project config (`/physics/`, `/viewer/`, `/network/`, `/scene/`, `/field/` — world-level,
the same for every robot) with that robot's own `/robot/*` and `/camera/*` tree merged in (both
already lived in `robot.json`, not `project.json`, so no config file needed to change shape at
all). `App::init` now only loads `project.json` into the singleton; loading `robot.json` moved from
there into `addRobot`.

Nothing needed to change in `Robot`/`Camera`/`LidarSensor`/`Dribbler`/`Kicker`/`ImuSensor`
themselves — they already treated the `Config&` they were handed as "this robot's config," not
necessarily *the* global singleton; multi-robot support only required making that assumption true
for more than one robot at a time.

## Spawn placement

`Robot::init` reads `/robot/spawn/x`, `/robot/spawn/z` (mm) and `/robot/spawn/yaw_deg` (deg),
default `0,0,0` — applied to the rigid body's own `startTransform` (origin + a `btQuaternion`
about Y) and the initial `m_position`/`m_yaw`, so multiple robots don't all spawn stacked at the
origin. The quaternion angle is used directly (not negated, unlike the per-wedge chassis-notch
rotation elsewhere in `Robot::init`): `syncFromPhysics`'s `atan2(-fwd.z(), fwd.x())` recovers
exactly `spawnYawRad` back out of a Y-axis `btQuaternion(spawnYawRad)`, verified by construction
from the documented Bullet-vs-this-codebase yaw sign relationship in `applyDriveForces`'s own
comment (`AGENTS.md`'s yaw convention section derives the same relationship for the wedge shapes).

Two ways to place a team:
- Give each robot its own config file, each with its own `/robot/spawn/*` (different hardware
  *and* position — e.g. genuinely different opponent robots).
- Call `App::addRobot(path, spawnX, spawnZ, spawnYawDeg)` with explicit spawn arguments — these
  win over (are merged on top of, via `Config::setRobotSpawn`) whatever the file itself says,
  letting several robots share one config file without overlapping. `main.cpp`'s
  `--robot-spawn <x> <z> <yaw>` flag (applies to the *next* `--robot`) exposes this from the CLI.

Calling `addRobot` with the same config file and no spawn override twice **does** stack both
robots at the same point — this is a deliberate non-choice (no silent auto-spacing), since a
config file's own explicit `/robot/spawn/*` must always be respected when the caller doesn't ask
for an override.

## gRPC: one `SharedState` per robot, not one shared struct

`GrpcServer` previously held a single `SharedState` and single `Robot*`/`Camera*`/etc. pointers.
Now: `std::map<int, RobotBinding> m_bindings` (raw pointers to that robot's subsystems) and
`std::map<int, SharedState> m_states` (one publish/command mutex-guarded buffer per robot),
populated by `registerRobot(id, ...)` (called once per `App::addRobot`) before `start()`.
`GrpcServer::update(dt)` loops over every binding: consumes that robot's pending command, and
(only when *that robot's own* `Camera::frameChanged()`) publishes its sensor frame — each robot's
`stream_fps` gates only its own publish rate, not anyone else's. `Ball` stays a single shared
instance (one ball on the field); its ground truth position is copied into every robot's own
`SensorData` on each publish.

`SimulatorServiceImpl` now takes `std::map<int, SharedState>&` instead of one `SharedState&`.
`SendCommand` looks up `cmd.robot_id()` per message (an unknown id fails that stream with an error
response, same as the old "only robot_id 0 exists" check); `SensorStream` looks up
`request->robot_id()` once at call start. Every lookup takes `GrpcServer::m_statesMutex` shared
first (`registerRobot` takes it exclusive to insert) — see "Dynamic robot creation" below for why
the map is no longer guaranteed fixed after `start()`, which is what would have made that lock
unnecessary. Only each `SharedState`'s own mutex guards its fields once found, unchanged from the
single-robot version; a `SharedState&`/pointer obtained under the shared lock stays valid to use
afterward without holding it, since `std::map` entries are never erased (only inserted) here, so
insertion elsewhere never invalidates an already-found reference.

## Keyboard: 1/2/3/4 select the active robot

`App::m_activeRobot` (an index into `App::m_robots`) picks which robot the manual keyboard
controls (arrows/Q/E drive, Space dribble, F debug-fire) and the camera-preview (`C`) / telemetry
(`L`) overlays act on. The 1/2/3/4 keys (`SDL_SCANCODE_1`..`SDL_SCANCODE_4`) set it, clamped to
however many robots actually exist (a no-op past that). Manual-drive/dribble/kick edge-state
(`lastManualDrive`, `lastManualDribbleHeld`, etc.) moved from `App` members to per-robot fields on
`RobotEntry` — kept per-robot specifically so switching the active robot mid-session doesn't
produce a spurious press/release edge on either robot involved (e.g. switching away from a robot
mid-turn doesn't send it a phantom "key released" `setBodyVelocity(0,0,0)`, and switching to a
robot that was mid-turn under manual control before doesn't re-trigger its last edge either).

## Rendering

Each robot's own mirror-camera capture pass (`Camera::renderView`) now also draws every *other*
robot as a real obstacle (`Robot::renderBody` — the same wireframe body+wheels the debug viewer
already used, not the stripped support-frame the camera owner draws for itself) — a vision-league
robot's camera should see teammates/opponents, not just the ball and its own understructure. This
is a small, clearly-scoped addition (still wireframe, not the solid/lit style `Ball`/`Field` use)
rather than a push for full multi-robot vision realism, which was out of scope here. The
third-person viewer draws every registered robot with `Robot::render` (full body + mirror +
camera indicator), same as it always drew the single robot.

## Dynamic robot creation (`Simulator.AddRobot` RPC)

`App::addRobot` is also reachable at runtime, after `App::run()` has started serving, over gRPC --
not just "in code" before it. Added because a Python control script naturally wants to build its
whole team at the top of the script and then just work with the robots, without needing to relaunch
the simulator with the right `--robot` flags first (see `python/robot_hal.py`'s `SimulatorClient`).

The wrinkle: robot construction (a Bullet rigid body, a `Camera`'s GL framebuffers) is only safe on
the main/sim thread, but the RPC arrives on a gRPC handler thread. So `AddRobot` doesn't construct
anything itself -- it pushes a `PendingAddRobot` (`shared_state.h`: config path + spawn pose + its
own `mtx`/`cv`/`done`/result fields) onto `GrpcServer::m_addRobotQueue` (guarded by its own mutex,
separate from `m_statesMutex`) and blocks on that request's own `cv` until `done`. `GrpcServer::
update()` drains the whole queue *first*, before its per-robot command/publish loop, each tick: for
every queued request it calls the callback `App::init` installed via `setAddRobotHandler` (a lambda
wrapping `addRobot`) on the main thread, fills in the request's result, sets `done`, and notifies --
unblocking the waiting RPC handler thread with the real result. Draining before the per-robot loop
(not during or interleaved with it) means a robot added this tick has its `m_bindings`/`m_states`
entries fully in place before anything tries to use them that same tick, rather than depending on
`std::map`'s range-for visiting a key inserted mid-iteration.

This turned `m_states` from "read-only after `start()`" into "can grow at any time" -- see the
gRPC section above for the `m_statesMutex` fix this required: `registerRobot` now takes it
exclusive to insert, and every lookup (`GrpcServer::update()`'s own per-robot loop included, not
just `SimulatorServiceImpl`'s RPC handlers) takes it shared first -- a concurrent `std::map` insert
vs. find from different threads is undefined behavior with no synchronization at all, even though
insertion alone never invalidates existing references.

Two more things this required, both real bugs caught during testing (not just theoretical):
- `Config::loadRobot` throws `std::runtime_error` on a missing/unreadable file. That was previously
  fine to leave uncaught (main.cpp's `App::init` config load, and a bad `--robot` path, would just
  abort the whole process at startup -- acceptable for a CLI typo). Once the same `addRobot` path
  also runs mid-session from an `AddRobot` RPC, an uncaught exception there would crash the entire
  simulator -- every other robot's session too -- over one bad request from one client. `App::
  addRobot` now catches it, logs, and returns `-1` (which `AddRobot`'s response reports as
  `success=false` with the exception's message) instead of propagating. Verified: an `add_robot()`
  call with a nonexistent path raises a clean `RuntimeError` in the Python client while the
  simulator process (and any already-added robots) keeps running.
- `GrpcServer::stop()` now also fails (not just silently drops) any requests still sitting in
  `m_addRobotQueue` -- `update()` won't run again to drain them once the sim loop has stopped, and
  their handler threads are blocked waiting on `req->done`, which would otherwise hang `m_server->
  Shutdown()` (it waits for in-flight RPCs to finish).

`main.cpp`'s `--empty` flag skips the usual "no `--robot` flags given -> add one default robot"
fallback, for exactly this workflow: launch the simulator with an empty field, then build the team
entirely from Python via `SimulatorClient.add_robot()`.

## Known simplifications

- No auto-spacing for robots added from the same config file with no spawn override (see above) —
  a deliberate choice to never silently override a config file's own stated position.
- Every robot currently shares the one field-wide `Ball`; there's still only one ball on the
  field, as in the real 2v2/3v3 Vision leagues this simulator targets.
- No `RemoveRobot`/despawn RPC -- a robot, once added, stays for the simulator process's lifetime.
