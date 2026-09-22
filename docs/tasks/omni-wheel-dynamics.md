# Task: 3-wheel omnidirectional drive with realistic slip/friction

This is a self-contained implementation prompt for an AI coding agent (or a human) working on
this repo. It replaces the current 2-wheel differential-drive placeholder with a physically
simulated 3-omni-wheel drivetrain: real mass/inertia, per-wheel Coulomb friction against the
floor, motor lag, and a separately-drifting dead-reckoning odometry estimate — so control code
tested against this simulator has to cope with the same imperfections a real robot has (slip,
momentum, drift), not an idealized kinematic robot.

Read [`AGENTS.md`](../../AGENTS.md) and [`ROADMAP.md`](../../ROADMAP.md) first. This task
implements roadmap item 4 ("Robot ↔ physics integration"), resolved in favor of real dynamics,
and changes the `RobotCommand`/`SensorData` proto shape — read the whole file before touching
code, the wire format changes are load-bearing for later phases.

## Scope

**In scope:** everything needed to go from "robot moves by teleporting its position each frame"
to "robot is a real Bullet rigid body, driven by 3 omni-wheels whose ground force is limited by
friction, so aggressive commands cause visible slip/drift instead of perfect tracking," plus the
external interface changes (`vx`/`vy`/`omega` body-frame commands, noisy odometry in
`SensorData`) and updating the in-app manual drive to match.

**Out of scope — do not touch, even if it seems related:**
- Ball, kicker/dribbler effects, field wall/goal collision shapes, multi-robot support. Separate
  `ROADMAP.md` items. The robot can currently drive through field boundaries with no collision —
  that's item 7, not this one.
- Motor electrical modeling (current limits, back-EMF, PWM) — the existing first-order lag
  (`motor_time_constant`) stays the only motor model, just generalized from 2 wheels to 3.
- Weight-transfer / tipping dynamics (e.g. more load on the front wheel under braking) — assume
  the robot's weight splits evenly across the 3 wheels at all times.
- Advanced tire/contact models (Pacejka, etc.) — plain Coulomb friction (force capped at μ×N) is
  the target fidelity here, deliberately.
- The mirror-camera rendering pipeline (`Camera`, `MirrorProfile`) — it already reads
  `Robot::position()`/`orientation()` each frame and needs no changes; just don't break that
  contract (see Phase 1).

## Decisions already made (don't re-litigate these)

1. **Physics approach: single Bullet rigid body for the chassis + a hand-written analytical
   per-wheel friction-force model**, not three separate wheel sub-bodies with constraints, and
   not `btRaycastVehicle` (built for car-like wheels, not omni-wheels) or Bullet's
   `setAnisotropicFriction` (would require the chassis shape itself to be the contact surface,
   awkward for 3 discrete contact points at different angles). The robot chassis is one
   `btRigidBody`; each physics tick we compute a friction force per wheel by hand and apply it
   with `applyForce` at that wheel's contact point. Rationale: keeps all the physics in one place
   in `Robot`, easy to tune and debug, no small-body constraint-solver jitter to fight.
2. **External command shape: body-frame `vx`/`vy`/`omega`.** `RobotCommand` no longer carries
   per-wheel targets — callers command a velocity in the robot's own frame (`vx` forward,
   `vy` lateral, `omega` yaw rate) and the omni kinematics live inside the simulator. This is
   also how real omni-robots are normally commanded from a vision/strategy layer.
3. **Wheel layout: 3 wheels, 120° apart, tangential rolling direction** (standard omni-wheel
   config — each wheel drives tangentially to the mounting circle and free-slides radially).
   Default mounting angle offset `90°` for wheel 0 (so no wheel sits exactly on the forward
   axis) — configurable, not hardcoded.
4. **`SensorData` keeps ground-truth pose** (as today) **and gains a separate, realistically
   drifting odometry estimate.** Ground truth is what the simulator/renderer/graders use; the
   new odometry fields are what a real robot would actually have (dead-reckoning from wheel
   speeds, blind to slip) — the gap between them *is* the thing this task makes real.

## The math: 3-omni-wheel kinematics

Local robot frame (matches the rest of the codebase — see `MirrorProfile`/`Camera`): `+X` is
robot-forward, `+Z` is the other ground-plane axis (call it "lateral"), `+Y` is up, yaw rotates
`+X` toward `+Z` for positive `omega` (this matches the sign convention already validated for
`m_yaw` in `Robot::update` today — don't flip it).

Wheel `i` (i = 0,1,2) sits at angle `θ_i = θ0 + i·120°` around the robot center, at radius `R`
(= existing `/robot/wheels/center_diameter` / 2), where `θ0` defaults to 90° (config override,
see Phase 0). Wheel `i`'s position relative to the robot center:

```
p_i = R · (cos θ_i, 0, sin θ_i)
```

Its **tangential (driven/rolling) direction** and **radial (free/lateral) direction**, both unit
vectors in the local XZ plane:

```
t_i = (-sin θ_i, 0, cos θ_i)   // rolling direction (motor-driven)
n_i = ( cos θ_i, 0, sin θ_i)   // radial direction (passive rollers, mostly free)
```

**Forward kinematics** (body velocity → desired wheel surface speed, no slip assumed):

```
wheelTarget_i = -vx·sin(θ_i) + vy·cos(θ_i) + ω·R
```

Derivation: ground-contact-point velocity for wheel `i` is `(vx, vy) + ω·(-p_i.z, p_i.x)`
(2D rotation term), dotted with `t_i`. Do the algebra once, unit-test it if you want, but the
formula above is the answer — use it directly rather than re-deriving per call site.

**Inverse kinematics** (wheel speeds → estimated body velocity, used for odometry): build the
3×3 matrix `M` whose row `i` is `(-sin θ_i, cos θ_i, R)`, invert it once at init (angles are
fixed), and compute `(vx, vy, ω) = M⁻¹ · wheelSpeeds` each time you need an estimate. Don't
hand-solve a closed form — just invert the matrix (glm has `glm::inverse` for a `mat3`).

## Implementation plan

Work through these phases in order; each should leave the project building and running. Commit
after each phase (see `CONTRIBUTING.md`).

### Phase 0 — config

Add to `robot.json`:
```json
"wheels": {
    "count": 3,
    "center_diameter": 160,
    "wheel_diameter": 48,
    "angle_offset": 90
},
"physics": {
    "wheel_friction_driven": 0.9,
    "wheel_friction_lateral": 0.15,
    "moment_of_inertia_z": null
}
```
(`angle_offset` in degrees; `moment_of_inertia_z` in kg·m² — `null`/absent means "compute from
mass and radius," see Phase 1. Put the new `physics` block under `/robot/physics` since it's
robot-specific, not the top-level `/physics` in `project.json` which is field/ball/global.)

`project.json`'s existing `/physics/robot/mass` (currently `2500`) and `/physics/robot/friction`
/`restitution` — **note the mass value is grams, not millimetres**, even though it's converted
with the same `/ 1000` pattern as lengths (grams→kg happens to use the same divisor as mm→m;
they're different physical quantities, don't let that confuse the conversion code — comment it).
Keep `friction`/`restitution` as the chassis's own Bullet collision-surface friction (see Phase
1 — this should end up near-zero, since in reality only the wheels touch the ground, not the
chassis belly) rather than reusing it for wheel friction; the two new
`wheel_friction_driven`/`wheel_friction_lateral` keys are what drive the actual traction model.

### Phase 1 — robot as a Bullet rigid body

- `Robot` gains a `btRigidBody` (cylinder collision shape, radius/height from existing
  `m_diameter`/`m_height`), created in `Robot::init` and registered into the world — `Robot`
  needs a `Physics&` (or `btDiscreteDynamicsWorld*`) passed into `init` for this; thread it
  through `App::init` (`Physics` is already constructed before `Robot` there).
- Mass from `/physics/robot/mass` (grams → kg). Moment of inertia: if
  `/robot/physics/moment_of_inertia_z` is present use it, else default to solid-cylinder
  `0.5 · mass · (diameter/2)²`. Bullet wants a full inertia vector
  (`btVector3(ix, iy, iz)`) — for a cylinder standing upright, `iy` (about the vertical axis) is
  the one that matters for turning; the horizontal-axis inertias barely matter since angular
  motion there is about to be locked anyway (see below), reasonable to reuse the same computed
  scalar for all three or use Bullet's own `btCylinderShape::calculateLocalInertia`.
- **Lock tipping**: `body->setAngularFactor(btVector3(0, 1, 0))` — only allow rotation about Y
  (yaw). Otherwise nothing stops the friction forces below from tipping the robot over, which
  isn't a thing this project needs to simulate.
- **Chassis friction/damping**: set the chassis's own Bullet surface friction very low
  (`/physics/robot/friction`, recommend defaulting that config value to something small like
  `0.05` now that it means "belly-on-ground drag," not "traction" — traction comes entirely from
  the per-wheel model in Phase 3) and `body->setDamping(0, 0)` — otherwise Bullet's own contact
  friction and/or linear/angular damping will fight the hand-rolled wheel friction model and
  you'll get mystery extra drag that's a pain to debug. All horizontal drag should come from one
  place (Phase 3), not two.
- Keep the ground plane collision (already exists in `Physics`) for vertical support — you're
  not replacing that, just adding the robot as a second (dynamic) body in the same world.
- **Remove** the old kinematic integration in `Robot::update()` entirely (the
  `m_position.x += ...`/`m_yaw += ...` lines) — position/yaw now come from the Bullet body's
  transform after each physics step (`body->getWorldTransform()`), read back into `m_position`/
  `m_yaw` so `Robot::render`/`renderBody` (used by the mirror-camera cubemap capture — don't
  change their signatures) keep working unmodified.
- **Ordering in `App::update`**: compute and apply this frame's wheel friction forces (Phase 3)
  *before* `m_physics->step(dt)`, then read the robot's new transform back out *after* the step.
  Today's order is `m_physics->step(dt); m_robot->update(dt);` — that needs to become roughly
  `m_robot->applyDriveForces(dt); m_physics->step(dt); m_robot->syncFromPhysics();` (naming your
  call, just keep the apply-before-step / read-back-after-step ordering explicit).

**Acceptance:** project builds; with zero commanded velocity the robot just sits on the ground
under gravity without drifting or jittering (sanity check that the rigid body + angular lock +
damping settings are sane before adding any drive force).

### Phase 2 — wheel targets (kinematics + motor lag)

- Replace `Robot::setWheelVelocities(left, right)` with something like
  `Robot::setBodyVelocity(float vx, float vy, float omega)` storing the 3 commanded body
  values (clamp to sane magnitude bounds — reuse/rename the existing `m_maxSpeed`-style config
  if it still makes sense, or add explicit `/robot/motor/max_linear_speed` and
  `/robot/motor/max_angular_speed`, your call).
- Each physics tick, compute `wheelTarget_i` for the 3 wheels via the forward-kinematics formula
  above, then apply the existing first-order motor lag
  (`alpha = dt / (motor_time_constant + dt)`, blend toward target) **per wheel**, generalizing
  the current 2-channel blend in `Robot::update` to 3 channels. Store the result as
  `m_wheelSpeed[3]` — this lagged value is both (a) the "desired ground-contact rolling speed"
  fed into the friction model in Phase 3, and (b) the "encoder reading" fed into odometry in
  Phase 4. Real motors track their commanded speed with this kind of lag *before* any wheel
  slip happens at the ground — that's exactly why this value, not the raw target, is the right
  thing to treat as "what the motor/encoder says."

**Acceptance:** no visible behavior change yet (this phase only computes `m_wheelSpeed`, nothing
consumes it until Phase 3) — just confirm it builds and the values look sane if you log them.

### Phase 3 — per-wheel Coulomb friction (this is the actual "non-ideal" part)

Each physics tick, for each wheel `i`, in the robot's current local frame:

1. Get the robot's **actual** current body-frame velocity — the true one from the Bullet body
   (`getLinearVelocity()`/`getAngularVelocity()`, rotated into local frame using the body's
   current orientation), not the commanded one.
2. Compute the wheel's actual ground-contact velocity components using the same structure as the
   forward-kinematics formula, but with the *actual* `(vx, vy, ω)`:
   `actualRolling_i = -vx·sin θ_i + vy·cos θ_i + ω·R`, and similarly project onto `n_i` for
   `actualLateral_i` (desired lateral speed is always 0 — the wheel isn't driven sideways).
3. Slip = `m_wheelSpeed[i] - actualRolling_i` (rolling direction), and `0 - actualLateral_i`
   (lateral direction).
4. Per-wheel normal force: `N_i = (mass · g) / 3` (even weight split, per the out-of-scope note
   above — `g` from `/physics/gravity`, same conversion as elsewhere).
5. Force needed to fully close each slip gap this step: `F = (mass/3) · slip / dt` — clamp its
   magnitude to the Coulomb limit `μ · N_i` (`wheel_friction_driven` for the rolling axis,
   `wheel_friction_lateral` for the radial axis). Small slip → the clamp never triggers → it
   behaves like static friction (fully cancels the slip this step). Large slip (e.g. commanding
   more acceleration than the floor can support) → the clamp triggers → constant-magnitude
   kinetic friction opposing the slip, exactly the Coulomb model, and the wheel visibly can't
   keep up with the command. This is deliberately simple (no LCP solve) — it's stable and gets
   the right qualitative behavior without needing a real contact solver.
6. Total force for wheel `i` (local frame) = `forceRolling_i · t_i + forceLateral_i · n_i`;
   rotate into world space by the robot's current yaw; apply with
   `body->applyForce(worldForce, worldContactOffset)` where `worldContactOffset` is `p_i`
   (rotated by yaw) — Bullet converts the off-center application point into the correct combined
   force + torque on the body automatically, don't compute torque by hand.

**Acceptance:** this is the important one — after this phase, driving behavior should visibly
change under stress:
- Commanding a large `vx` instantly from rest: the robot accelerates smoothly over some time
  (limited by available friction) rather than snapping to target velocity — compare against
  `wheel_friction_driven` set very low (e.g. `0.05`) in a scratch config: the robot should barely
  move, wheels effectively spinning in place.
- Commanding forward motion `vx` plus a strong `omega` at the same time (fast turn while moving)
  should show visible drift/understeer compared to the commanded path, not a perfect arc.
- With `omega` alone (spin in place, no `vx`/`vy`), behavior should look close to the old
  differential-drive spin test from the mirror-camera work (see chat history) — small slip, not
  a big qualitative change, since spin-in-place is the easiest case for 3 symmetric wheels.

### Phase 4 — dead-reckoning odometry

- New small piece of state (in `Robot`, or a tiny helper struct): `odomX`, `odomZ`, `odomYaw`,
  initialized to the robot's starting pose.
- Each physics tick, run the **inverse kinematics** (the `M⁻¹ · wheelSpeeds` from the math
  section) on `m_wheelSpeed[3]` (the lagged/commanded values from Phase 2 — *not* the true
  Bullet velocity) to get an estimated body-frame `(vx_est, vy_est, ω_est)`. Integrate:
  `odomYaw += ω_est · dt`, then advance `odomX`/`odomZ` by rotating `(vx_est, vy_est)` into world
  space using `odomYaw` (its own accumulated yaw — not the ground-truth yaw) and scaling by `dt`.
- This estimate silently assumes zero slip (exactly what a real robot's onboard dead-reckoning
  does) — whenever Phase 3's friction model actually limits the wheels, `odomX/odomZ/odomYaw`
  will drift away from the real `Robot::position()/orientation()`. That gap is the point.

**Acceptance:** log both ground truth and odometry pose while doing a hard acceleration or tight
turn; confirm they diverge, and confirm they stay identical (up to floating point) when driving
gently within the friction budget.

### Phase 5 — external interface

- `simulator/proto/simulator.proto`:
  - `RobotCommand`: replace `float left_wheel = 2; float right_wheel = 3;` with
    `float vx = 2; float vy = 3; float omega = 4;`, renumber `kick_power`/`dribble_speed`/
    `timestamp` to `5`/`6`/`7` (no external consumers besides this repo's own code yet, so a
    clean renumber is fine — don't try to preserve old field numbers for compatibility that
    doesn't exist).
  - `SensorData`: add `float odom_x = 12; float odom_z = 13; float odom_yaw = 14;` after the
    existing `timestamp = 11`.
  - Regenerate C++ stubs (CMake does this automatically on build) and Python stubs
    (`bash python/generate_proto.sh`).
- `SimulatorServiceImpl::SendCommand`: read `cmd.vx()/cmd.vy()/cmd.omega()` and call
  `robot->setBodyVelocity(...)` instead of `setWheelVelocities`.
- `SimulatorServiceImpl::SensorStream` / `SharedState`: publish the new odometry fields
  alongside the existing ground-truth ones each tick.
- `App::handleKeyboardInput` (manual drive): rewrite to drive `vx`/`vy`/`omega` directly instead
  of left/right wheel targets:
  - Up/Down arrows → `vx` (unchanged mapping, still robot-forward by construction now)
  - Left/Right arrows → `omega` (unchanged mapping)
  - Add `Q`/`E` for `vy` (strafe) — new capability, `WASD`/arrows/Shift/Cmd/`C` are all already
    taken, `Q`/`E` are free and match the common game "strafe" binding. Document in `README.md`.
  - Keep the existing press/release-edge-only dispatch (don't resend every frame — see the
    comment already in `app.h`/`app.cpp` explaining why: doesn't fight an active gRPC client).
- Double-check nothing else in the repo references `left_wheel`/`right_wheel` (grep before
  finishing) — `python/viewer.py` doesn't construct `RobotCommand` at all, so it shouldn't need
  changes, but verify.

**Acceptance:** drive the robot with arrow keys + Q/E in the simulator window and confirm all
three axes work independently and in combination; then repeat via a small Python test script
(open a `SendCommand` stream, send a few `RobotCommand`s with `vx`/`vy`/`omega`) — same pattern
as the manual spin test used to validate the mirror-camera work — and confirm `SensorStream`
reports both ground-truth pose and the new `odom_x/odom_z/odom_yaw` fields, diverging under hard
commands as expected from Phase 4.

### Phase 6 — wrap up

- Update `ROADMAP.md`: mark item 4 done, note items 6/7 (multi-robot, field collision) are
  unaffected/still open, and add a one-line pointer to this file for how item 4 was implemented
  (same pattern as items 1/2 pointing at `docs/tasks/mirror-camera-vision.md`).
- Update `AGENTS.md`'s class list only if you introduced something worth keeping consistent
  (e.g. document the local-frame convention — forward `+X`, lateral `+Z`, CCW-positive yaw — if
  it isn't written down anywhere else yet; it's used by both this task and the mirror-camera
  math, worth having in one place rather than re-explained per task file).
- Don't touch `Camera`/`MirrorProfile` — confirm the mirror-camera preview (`C` key) and gRPC
  `SensorStream` image still work unchanged after these changes (they depend on
  `Robot::position()`/`orientation()` staying accurate, nothing else).

## Notes for whoever reviews this

- Units: `robot.json`/`project.json` values are millimetres for lengths (existing convention)
  and — per Phase 0 — grams for mass; everything converts to SI (metres, kilograms, radians) once
  at load time, same as the rest of the codebase (see `AGENTS.md`).
- If slip/friction numbers feel arbitrary to tune, that's expected — pick physically plausible
  starting points (rubber-ish `wheel_friction_driven` around 0.8–1.0, a much lower
  `wheel_friction_lateral` around 0.1–0.2 to represent imperfect rollers) and leave them as easy
  config knobs rather than trying to nail "correct" values; the point is that the mechanism is
  real, not that the specific coefficients are calibrated against a real robot.
