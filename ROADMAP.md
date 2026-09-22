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

## Stubbed / not implemented

Ranked by what's most load-bearing for the project's actual purpose (feeding vision data to a
robot-control client) — do this roughly top to bottom, but treat it as a starting point to
argue with, not a mandate.

1. **gRPC server** (`GrpcServer` / `grpc_server.cpp`) — `start()`/`stop()`/`update()` only log
   to stdout. `proto/simulator.proto` already defines `SensorStream` (stream camera image +
   pose to client) and `SendCommand` (drive robot from client). CMake already links gRPC and
   generates the protobuf/grpc stubs. Nothing implements `Simulator::Service` yet — this is
   the actual external interface of the simulator and currently does nothing.

2. **Real mirror-camera rendering** (`Camera::update()`) — currently fills the image buffer
   with a synthetic noise pattern, ignores `robotPos`/`robotYaw` entirely. Needs to render the
   scene from the robot's camera position through the catadioptric mirror (`Camera::Mirror`:
   `a`, `b`, `radius` are already loaded from config) into `m_fbo`/`m_renderTexture`, then read
   the pixels back into `m_imageData`. This is what `SensorStream` is supposed to send.

3. **Ball** — there is no ball anywhere in the codebase (`Physics`, `Field`, `Robot`). Configs
   already carry `physics.ball` (radius/mass/friction/restitution) unused. Needs a `Ball` type
   (or similar) with a Bullet sphere rigid body, spawn position, and rendering.

4. **Robot ↔ physics integration** — `Robot::update()` moves the robot by directly integrating
   position/yaw; the Bullet world only has a ground plane and never knows the robot exists, so
   there's no collision between the robot, field walls, or a future ball. Decide explicitly
   whether the robot stays kinematic (simpler, deterministic, easier to match real robot specs)
   or becomes a real rigid body (needed for physical collisions) — don't let this drift
   silently once a ball exists.

5. **Kicker / dribbler** — `Robot::kick()` and `Robot::dribble()` are no-ops. Proto already
   carries `kick_power`/`dribble_speed`. Needs an actual effect once there's a ball to act on.

6. **Multi-robot support** — `App` holds a single `std::unique_ptr<Robot> m_robot`, but
   `SensorRequest`/`RobotCommand` already carry `robot_id`. If the target league needs more
   than one robot (own team and/or opponents) on the field, this needs a robot registry
   (e.g. `std::map<int, Robot>`) instead of a single instance.

7. **Field collision** — walls/goals are drawn (`Field::render`) but have no Bullet collision
   shapes, so nothing currently stops the robot or a future ball from leaving the field.

## Not urgent

- Extract the inline GLSL strings in `renderer.cpp` into `simulator/shaders/*.glsl` if shader
  code grows enough to be worth the indirection. The `shaders/` directory exists for this but
  is currently unused — don't add files there speculatively.
- `simulator/lib/` is currently empty and unused; only populate it if a dependency actually
  needs to be vendored.
- Confirm Linux build path (CMake has a non-Apple OpenGL branch that's never been exercised).
