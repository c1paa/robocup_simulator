# Task: Ball physics + real wall/goal collision response

This is a self-contained implementation prompt for an AI coding agent (or a human) working on
this repo. It adds a real ball (a golf ball, per RoboCup vision-league convention: orange,
~42.7mm diameter, ~46g) to the simulation as a proper Bullet rigid body, and gives the field
boundary walls and goal structures actual collision *response* (they currently only have
raycast-only hitboxes for the lidar — see `docs/tasks/lidar-sensor.md`), so the ball bounces off
them and the robot can push it around, and — as a direct consequence of making the walls solid —
the robot itself stops driving through the field boundary and goals too.

Read [`AGENTS.md`](../../AGENTS.md) and [`ROADMAP.md`](../../ROADMAP.md) first. This implements
roadmap item 3 ("Ball") and, as a side effect, resolves the wall/goal half of item 7 ("Field
collision") — robot-vs-ball and robot-vs-wall collision both fall out of giving the wall/goal
boxes real contact response, since that's a body-level Bullet flag, not something that can be
scoped to one other body without collision groups (this was discussed with the project owner
and deliberately decided in favor of just enabling it for everyone — see "Decisions already
made" below).

## Scope

**In scope:**
- A `Ball` class: a dynamic Bullet `btSphereShape` rigid body using the existing (currently
  unused) `/physics/ball/*` config values, standard Bullet dynamics (no hand-rolled force model
  needed — unlike `Robot`, a sphere rolling on a plane is exactly what Bullet's built-in
  friction/restitution already model correctly).
- Removing `CF_NO_CONTACT_RESPONSE` from the boundary-wall and goal-wall bodies in
  `Physics::init` (added in the lidar task) so they have real collision response, plus giving
  them explicit friction/restitution (currently unset, defaulting to Bullet's built-in
  defaults — not this project's configured values).
- Rolling friction / damping on the ball so it settles instead of rolling indefinitely (decided
  below).
- Rendering the ball (orange sphere) in both render passes — the main viewer window *and* the
  robot's own mirror-camera cubemap capture (see Phase 3; this second one is easy to miss and is
  the actual point of adding a ball to a vision simulator).
- Ground-truth ball pose in `SensorData` (for debugging/grading), alongside the existing robot
  ground truth.
- Config-driven masses for both robot and ball (robot's `/physics/robot/mass` already exists and
  is already used by `Robot`; ball's `/physics/ball/mass` already exists in config but is
  currently unused anywhere — wiring it up is part of this task, not a new config key).

**Out of scope — do not touch, even if it seems related:**
- Kicker/dribbler (`Robot::kick()`/`Robot::dribble()` stay no-ops) — that's roadmap item 5, a
  separate task that will act on the ball this task introduces, but isn't this task's job.
- Multi-robot support (item 6) — one robot, one ball.
- Any change to the robot's own drivetrain/friction model (`Robot::applyDriveForces`) — the
  robot pushing the ball around, and the robot now colliding with walls, both fall out of normal
  Bullet rigid-body dynamics once the ball exists and the walls have contact response; no
  special-casing needed in `Robot`.
- Collision groups/masks to make walls solid for the ball but not the robot — deliberately
  rejected; walls/goals are just solid for everyone now (see Decisions).
- Ball detection via the camera's rendered image (color/blob detection, etc.) — out of scope for
  the simulator itself; the simulator's job is just to make sure the ball is actually *in* the
  rendered image a client would need to run such detection against. What a robot-control client
  does with that image is downstream of this repo.

## Decisions already made (don't re-litigate these)

1. **Walls/goals become solid for both the robot and the ball, not just the ball.**
   `CF_NO_CONTACT_RESPONSE` is a flag on the wall/goal body itself and affects every other body
   that touches it — there's no way to make it solid for the ball but not the robot without
   introducing Bullet collision groups/masks, which is meaningfully more code for a
   distinction nobody asked for. Discussed with the project owner: robot-vs-wall collision
   (driving through the boundary) was already a known, tracked gap (roadmap item 7) and is
   reasonable to close as a side effect here, not a regression to design around.
2. **The ball is a plain Bullet `btSphereShape` dynamic body, using stock Bullet
   friction/restitution/rolling-friction — no custom per-contact force model.** This is
   deliberately different from `Robot`, which needed a hand-rolled Coulomb model because omni
   wheels aren't something Bullet has a built-in concept of. A round ball rolling and bouncing
   is exactly the case Bullet's built-in rigid-body dynamics already handle correctly.
3. **Rolling friction / damping is added** (`btRigidBody::setRollingFriction` and/or
   `setDamping`) so the ball settles to a stop after rolling, rather than coasting indefinitely
   — plain Coulomb sliding friction alone doesn't dissipate a rolling sphere's energy in Bullet
   (rolling ≠ sliding), so without this the ball would roll for a very long time after any push.
   Tune to taste, but it should visibly settle within a few seconds after a moderate push, not
   in 200ms (dead-feeling) or 30s (never-ending).
4. **Energy loss on bounce comes from the ball's configured `restitution` (already `0.8` in
   `physics.ball`), combined with new explicit friction/restitution values on the wall/goal
   bodies** (currently unset — Bullet's constructor defaults, not this project's numbers).
   Bullet's default contact-restitution combine mode is `max(body_a, body_b)` — be aware of this
   when picking wall/goal restitution values; if walls end up with restitution `1.0` the ball
   would visibly gain height on every bounce (obviously wrong physically), so pick a value ≤ the
   ball's own if choosing a fixed value, or verify empirically what "loses energy on every
   bounce" actually requires from this specific combine mode rather than assuming.
5. **Spawn position: field center**, resting height (`radius + tiny epsilon` above the ground
   plane so it doesn't start interpenetrating). No reset-on-goal or scoring logic — that's a
   future task once there's a reason to reset (goal detection, refereeing, etc.), not this one.
6. **No angular-factor locking on the ball** (unlike `Robot`, which locks `setAngularFactor(0,1,0)`
   to prevent tipping) — a ball is supposed to roll and tumble freely in 3D; let Bullet handle
   full rotation.

## Config (already exists — verify these are the values you're actually using, don't reinvent)

`simulator/configs/project.json` already has:
```json
"physics": {
    "ball": {
        "radius": 21.5,
        "mass": 0.046,
        "friction": 0.07,
        "restitution": 0.8
    },
    "robot": {
        "mass": 2500,
        "friction": 0.0,
        "restitution": 0.0
    }
}
```
(radius/mass in mm/g per this project's config convention — convert once at load time, same
pattern as everywhere else; `physics.ball.radius` is in **mm** like every other length in this
config, so `21.5mm` radius = `43mm` diameter, matching a real golf ball almost exactly, and
`0.046kg` matches a real golf ball's mass — this was clearly set up in anticipation of this
task, use it as-is rather than picking your own numbers).

**New config needed** — wall/goal material, since nothing currently specifies it (add under
`/physics/field/`, reusing the same block the ground plane already reads
`friction`/`restitution` from — see `Physics::init`):
```json
"physics": {
    "field": {
        "friction": 0.8,
        "restitution": 0.5,
        "wall_friction": 0.3,
        "wall_restitution": 0.4
    }
}
```
Reused for *both* the boundary walls and the goal walls (they already share the same box-creation
code path in `Physics::init` — keep it that way, don't fork the logic per wall type).

**New config needed** — ball color (add under `/physics/ball/` or `/field/`, your call, but
match this project's existing convention of `color/0,1,2` array-of-floats like
`field.floor_color`/`wall_color`):
```json
"color": [1.0, 0.45, 0.0]
```
(orange — RoboCup vision-league standard ball color.)

**New config needed** — rolling resistance:
```json
"rolling_friction": 0.02,
"linear_damping": 0.05,
"angular_damping": 0.1
```
(starting points, not gospel — tune during Phase 2 testing until the ball visibly settles in a
few seconds rather than never or instantly.)

## Implementation phases

**Phase 1 — real wall/goal collision.** In `Physics::init` (`simulator/src/physics.cpp`), remove
`btCollisionObject::CF_NO_CONTACT_RESPONSE` from both wall-creation loops (currently around
lines 99–127 — one loop for the four boundary walls, one for the six goal-structure boxes; both
set the same two flags, `CF_STATIC_OBJECT | CF_NO_CONTACT_RESPONSE` — keep `CF_STATIC_OBJECT`,
drop only `CF_NO_CONTACT_RESPONSE`), and set `ci.m_friction`/`ci.m_restitution` from the new
`/physics/field/wall_friction`/`wall_restitution` config keys before constructing each
`btRigidBody`. Update the comments on those loops and on `m_wallShapes` in `physics.h` — they
currently say "raycast targets only... full collision response is still ROADMAP item 7", which
will no longer be true.

Test this phase in isolation before moving on: drive the robot (via gRPC or the manual keyboard
controls) straight into a boundary wall and confirm it now stops instead of passing through.
This is a real behavior change to existing robot movement, not just new ball code — verify it
doesn't break the existing omni-wheel driving tests from `docs/tasks/omni-wheel-dynamics.md`
(forward/backward/strafe/turn away from any wall should behave exactly as before; only driving
*into* a wall changes).

**Phase 2 — `Ball` class.** New `simulator/src/ball.h`/`.cpp` (one class per file pair, per
`AGENTS.md`). Mirrors `Robot`'s general shape (owns its own `btCollisionShape`/
`btDefaultMotionState`/`btRigidBody`, an `init(Config&, btDiscreteDynamicsWorld*)`, a
`syncFromPhysics()` to read position/orientation back after each physics step, and `render()`).
Much simpler than `Robot` — no per-frame force application, no odometry, just a passive rigid
body Bullet drives on its own. Test with the existing physics debug-draw overlay
(`App::m_showPhysicsDebug`, already renders sphere shapes — see `Physics::debugDraw`'s
`SPHERE_SHAPE_PROXYTYPE` case) before wiring up dedicated ball rendering, so you can visually
confirm it drops, rolls, and bounces correctly with zero new rendering code.

**Phase 3 — rendering, in both passes.** `App::render()` (`simulator/src/app.cpp`) has two
separate scene-draw call sites that must **both** draw the ball, and it's easy to only do one:
- The main viewer window, around line 344–345 (`m_field->render(...)`, `m_robot->render(...)`).
- The robot's own mirror-camera cubemap capture, around lines 318–322 — a lambda passed to
  `m_camera->renderView(...)` that currently draws `m_field->render(r)` and
  `m_robot->renderBody(r)`. **This second one is the actual point of adding a ball to a vision
  simulator** — if the ball isn't drawn into the cubemap capture, it's invisible to the robot's
  own camera feed and the whole exercise is pointless (the ball would only ever show up in the
  developer's own viewer, never in what a vision client actually receives over gRPC). Add
  `m_ball->render(r)` to both call sites.

**Phase 4 — proto/config wiring.** Add ball ground-truth fields to `SensorData` in
`simulator/proto/simulator.proto` (`ball_pos_x`/`ball_pos_y`/`ball_pos_z`, next available field
numbers), populate them through the same `SharedState` → `SimulatorServiceImpl::SensorStream`
bridge pattern already used for the robot's pose and the lidar scan. Regenerate Python stubs
(`python/generate_proto.sh`) and add a `get_ball_position()` accessor to
`python/robot_hal.py::SimRobotHAL`, matching its existing method-naming style.

## Testing

Empirical, gRPC-driven testing (this repo's established approach — see the omni-wheel and lidar
task histories, where hand-derived sign/geometry reasoning produced real bugs multiple times and
direct measurement caught them). At minimum verify: the ball settles at rest under gravity on
the flat ground without jittering; a push (apply an initial velocity, or drive the robot into it)
sends it rolling and it visibly slows to a stop within a few seconds, not instantly and not
"forever"; it bounces off a boundary wall and loses height/speed on each bounce (successive
bounce peak heights should decrease, not stay constant or increase); the robot driving into the
ball pushes it (given the ~54:1 mass ratio — 2.5kg robot vs 46g ball — this should be easy and
not require any tuning); the robot driving into a wall now stops instead of passing through
(Phase 1's regression check). Also render a frame from the robot's own mirror-camera preview
(`App::m_showCameraPreview`, or pull a frame via `python/viewer.py`/`SensorStream` gRPC) with the
ball placed somewhere clearly visible and confirm it's actually there — this is the one check
that's easy to skip and would silently defeat the point of Phase 3.

## After implementing

Update `ROADMAP.md` (mark item 3 done, and note in item 7 that wall/goal collision response is
now also done — only remaining item-7 gap, if any, would be anything not covered by the boxes
already added for the lidar task) and `AGENTS.md` (add `Ball` to the class list, note the new
`/physics/field/wall_*` and `/physics/ball/*` config keys in the conventions section if they
need explanation beyond what's already documented), the same way prior task docs describe their
own landing.
