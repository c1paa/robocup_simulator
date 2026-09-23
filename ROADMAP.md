# Roadmap

A snapshot of what actually works today vs. what's stubbed, and a suggested order to tackle
the rest. Update this file as items land — it's the shared source of truth for "what's next,"
for humans and AI agents alike.

## Working today

- SDL2 + OpenGL 3.3 core window with orbit/pan/zoom viewer camera (`App`)
- Field geometry and line/goal/wall rendering from config (`Field`)
- Single robot: a real Bullet rigid body driven by a 3-omni-wheel drivetrain
  with per-wheel Coulomb friction/slip, motor lag, and a separately-drifting
  dead-reckoning odometry estimate (`Robot`) — commanded in body-frame
  `vx`/`vy`/`omega`
- Bullet physics world with a ground plane + the robot chassis (`Physics`)
- Config loading from JSON with dot-path lookup and mm→m conversion (`Config`)
- gRPC server (`GrpcServer` + `SimulatorServiceImpl`): `SensorStream` (camera image + pose
  + odometry) and `SendCommand` (drive commands) over `localhost:<grpc_port>`
- Real catadioptric mirror-camera rendering (`Camera` + `MirrorProfile`): cubemap capture +
  baked direction LUT → real, mirror-distorted image, streamed to clients

## Stubbed / not implemented

Ranked by what's most load-bearing for the project's actual purpose (feeding vision data to a
robot-control client) — do this roughly top to bottom, but treat it as a starting point to
argue with, not a mandate.

Items 1, 2, 3, 4, 5 and 8 are **done** — see [`docs/tasks/mirror-camera-vision.md`](docs/tasks/mirror-camera-vision.md)
(items 1–2), [`docs/tasks/ball-physics.md`](docs/tasks/ball-physics.md) (item 3, plus the
wall/goal half of item 7), [`docs/tasks/omni-wheel-dynamics.md`](docs/tasks/omni-wheel-dynamics.md)
(item 4), [`docs/tasks/dribbler-kicker.md`](docs/tasks/dribbler-kicker.md) (item 5) and
[`docs/tasks/lidar-sensor.md`](docs/tasks/lidar-sensor.md) (item 8) for how they
were implemented.

1. ~~gRPC server~~ — done (see above).
2. ~~Real mirror-camera rendering~~ — done (see above).

3. ~~Ball~~ — done: a `Ball` type with a Bullet sphere rigid body (spawned forward of the
   robot at field center), rolling friction/damping so it settles, rendered in both the viewer
   and the robot's mirror-camera cubemap, and streamed over gRPC as ground-truth
   `ball_pos_*`. As a side effect this gave the wall/goal boxes real collision response (they
   were raycast-only), closing the wall/goal half of item 7. See
   [`docs/tasks/ball-physics.md`](docs/tasks/ball-physics.md).

4. ~~Robot ↔ physics integration~~ — done: the robot is a Bullet rigid body driven by a
   3-omni-wheel friction/slip model, with body-frame `vx`/`vy`/`omega` commands and a
   separately-drifting odometry estimate. See
   [`docs/tasks/omni-wheel-dynamics.md`](docs/tasks/omni-wheel-dynamics.md).

5. ~~Kicker / dribbler~~ — done: the dribbler roller is a hand-rolled capture-force model
   (motor lag, load sag, tapered capture zone) that computes a single world-frame target
   velocity for the ball's contact point — rigid co-rotation with the robot, a spring-like pull
   to the center of the pocket, and the roller's own spin — all sharing one Coulomb-clamped grip
   budget, so a ball entering the zone snaps to dead center and stays there at rest, but a sharp
   enough turn or a hard reverse burst can genuinely eject it (keeping whatever spin it had).
   The kicker is a simulated-capacitor impulse. Robot chassis is a `btCompoundShape` (cylinder +
   a front "lip" that holds a resting, unpowered ball). Verified empirically via gRPC scripts:
   centered capture, bounded rest stability (no drift), hold-while-driving, turn/reverse
   ejection, kick height/torque, capacitor drain+recharge, forward/strafe regression. An earlier
   version of the force model (no centering, no rotation-awareness) had a real bug where a
   stationary captured ball would spontaneously eject after ~1s — fixed, not just mitigated; see
   "Force model revision" in [`docs/tasks/dribbler-kicker.md`](docs/tasks/dribbler-kicker.md)
   for what changed.

6. **Multi-robot support** — `App` holds a single `std::unique_ptr<Robot> m_robot`, but
   `SensorRequest`/`RobotCommand` already carry `robot_id`. If the target league needs more
   than one robot (own team and/or opponents) on the field, this needs a robot registry
   (e.g. `std::map<int, Robot>`) instead of a single instance. (Unaffected by item 4 — still
   open.)

7. **Field collision** — walls/goals are drawn (`Field::render`); the four boundary walls and
   both goal structures (side + back walls) have static collision boxes with full contact
   *response* for the robot/ball (no longer raycast-only) — see item 3. Any remaining gap, if
   any, would be field geometry not covered by the boxes already added for the lidar task.

8. ~~LiDAR sensor + Python hardware-abstraction seed~~ — done: an LD06-like 360° 2D scanning
   lidar (`LidarSensor`, Bullet raycast sweep with range/rate/noise/dropout matched to the real
   sensor) streamed over gRPC via `SensorData.lidar_points`, plus a `python/robot_hal.py`
   (`SimRobotHAL`) wrapping all sensor/command access. Added static raycast-only collision boxes
   for the four field boundary walls and both goal structures (a slice of item 7) as lidar
   targets — the lidar mount height (`robot.lidar.height`, default 50mm) is deliberately kept
   below `field.goal_height` (100mm) so the scan plane actually clips the goals instead of
   passing over them. See [`docs/tasks/lidar-sensor.md`](docs/tasks/lidar-sensor.md).

## Not urgent

- Extract the inline GLSL strings in `renderer.cpp` and `camera.cpp` into
  `simulator/shaders/*.glsl` if shader code grows enough to be worth the indirection. The
  `shaders/` directory exists for this but is currently unused — don't add files there
  speculatively.
- `simulator/lib/` is currently empty and unused; only populate it if a dependency actually
  needs to be vendored.
- Confirm Linux build path (CMake has a non-Apple OpenGL branch that's never been exercised).
