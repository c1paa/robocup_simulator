# Roadmap

A snapshot of what actually works today vs. what's stubbed, and a suggested order to tackle
the rest. Update this file as items land — it's the shared source of truth for "what's next,"
for humans and AI agents alike.

## Working today

- SDL2 + OpenGL 3.3 core window with orbit/pan/zoom viewer camera (`App`)
- Field geometry and line/goal/wall rendering from config (`Field`)
- Single robot: differential-drive kinematics (position/yaw integrate from wheel targets),
  cylinder+wheels+mirror rendering (`Robot`)
- Bullet physics world with a ground plane (`Physics`) — nothing else is in it yet
- Config loading from JSON with dot-path lookup and mm→m conversion (`Config`)
- gRPC server (`GrpcServer` + `SimulatorServiceImpl`): `SensorStream` (camera image + pose)
  and `SendCommand` (drive commands) over `localhost:<grpc_port>`
- Real catadioptric mirror-camera rendering (`Camera` + `MirrorProfile`): cubemap capture +
  baked direction LUT → real, mirror-distorted image, streamed to clients

## Stubbed / not implemented

Ranked by what's most load-bearing for the project's actual purpose (feeding vision data to a
robot-control client) — do this roughly top to bottom, but treat it as a starting point to
argue with, not a mandate.

Items 1 and 2 are **done** — see [`docs/tasks/mirror-camera-vision.md`](docs/tasks/mirror-camera-vision.md)
for how they were implemented.

1. ~~gRPC server~~ — done (see above).
2. ~~Real mirror-camera rendering~~ — done (see above).

3. **Ball** — there is no ball anywhere in the codebase (`Physics`, `Field`, `Robot`). Configs
   already carry `physics.ball` (radius/mass/friction/restitution) unused. Needs a `Ball` type
   (or similar) with a Bullet sphere rigid body, spawn position, and rendering.

4. **Robot ↔ physics integration** — `Robot::update()` moves the robot by directly integrating
   position/yaw; the Bullet world only has a ground plane and never knows the robot exists, so
   there's no collision between the robot, field walls, or a future ball. Decided: the robot
   becomes a real Bullet rigid body driven by a 3-omni-wheel friction/slip model (not kinematic).
   Full plan: [`docs/tasks/omni-wheel-dynamics.md`](docs/tasks/omni-wheel-dynamics.md) — this
   also changes `RobotCommand` (body-frame `vx`/`vy`/`omega` instead of `left_wheel`/
   `right_wheel`) and adds a separately-drifting odometry estimate to `SensorData`.

5. **Kicker / dribbler** — `Robot::kick()` and `Robot::dribble()` are no-ops. Proto already
   carries `kick_power`/`dribble_speed`. Needs an actual effect once there's a ball to act on.

6. **Multi-robot support** — `App` holds a single `std::unique_ptr<Robot> m_robot`, but
   `SensorRequest`/`RobotCommand` already carry `robot_id`. If the target league needs more
   than one robot (own team and/or opponents) on the field, this needs a robot registry
   (e.g. `std::map<int, Robot>`) instead of a single instance.

7. **Field collision** — walls/goals are drawn (`Field::render`) but have no Bullet collision
   shapes, so nothing currently stops the robot or a future ball from leaving the field.

## Not urgent

- Extract the inline GLSL strings in `renderer.cpp` and `camera.cpp` into
  `simulator/shaders/*.glsl` if shader code grows enough to be worth the indirection. The
  `shaders/` directory exists for this but is currently unused — don't add files there
  speculatively.
- `simulator/lib/` is currently empty and unused; only populate it if a dependency actually
  needs to be vendored.
- Confirm Linux build path (CMake has a non-Apple OpenGL branch that's never been exercised).
