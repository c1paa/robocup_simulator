# Dribbler + kicker mechanics

Implements the two remaining "field player" mechanisms: the spinning dribbler roller that
captures and holds the ball against the robot, and the solenoid kicker that fires it. Both are
currently no-op stubs (`Robot::kick()` / `Robot::dribble()` in `simulator/src/robot.cpp` just do
`(void)power;`), but the gRPC transport for them already exists end-to-end (`kick_power` /
`dribble_speed` in `RobotCommand`, wired through `SharedState` → `GrpcServer::update()` →
`Robot::kick`/`dribble`) and `robot.json` already has placeholder config sections
(`/robot/kicker`, `/robot/dribbler`) — this task fills in the physics behind them, not the
transport.

**Status: implemented, reviewed, and revised three times** — the original review found the capture
force model was missing an active centering pull and would spontaneously eject a stationary,
untouched ball after ~1s (see "Force model revision"). A later request added genuine pocket
recession (the ball sits physically recessed into the chassis, not flush against it — see
"Second revision"), which required shrinking the chassis's own collision geometry and removing
the original "lip". A third request replaced the single combined-direction Coulomb clamp with two
separately-budgeted axes (roller vs. cradle plate — see "Third revision"), so the "no separate
lateral allowance" language in the first revision below is superseded. Read all three revision
sections before touching `/robot/dribbler/friction`, `normal_force`, `centering_time_constant`,
`forward_offset`, or `pocket_depth`.

Read [`AGENTS.md`](../../AGENTS.md) first (units convention, yaw convention, config-access
pattern) and [`docs/tasks/ball-physics.md`](ball-physics.md) / `simulator/src/ball.cpp` for how
the ball is currently modeled — this task extends `Ball`, it doesn't replace it.

## How the real hardware works (for context, not for literal replication)

- **Dribbler**: a roller mounted horizontally in front of the robot, its axis parallel to the
  ground and perpendicular to the robot's forward direction (i.e. along the robot's local `Z`).
  A BLDC motor spins it fast (~6000 RPM on real hardware) so that the roller's surface at the
  point closest to the ball moves *toward the robot*. Friction against the ball's near-top
  surface imparts backspin; a backspinning ball rolling on the ground pulls itself back toward
  the robot on its own (the same effect that makes a heavily backspun golf ball "suck back" after
  landing) — this is what makes the ball cling to the robot rather than the roller having to
  physically block it. The roller's radius tapers down toward its center (two opposed
  cones/frustums, symmetric) so a free ball self-centers under gravity + the sloped surface. In
  front of the chassis there's a shallow scalloped ramp ("horka") that also helps hold the ball
  in place when the roller isn't spinning. Load from the ball touching the roller measurably
  slows the motor (more current draw / harder to spin at the same commanded voltage).
- **Kicker**: a solenoid plunger behind/below the dribbler, at a configurable height relative to
  the ball, driven by discharging a large capacitor (~20 000 µF at ~48 V) through it. Firing
  hits the ball with a large, very short impulse. Plunger height relative to the ball's center
  matters a lot: too low and the ball pops upward (chip), too high and it's pressed down into
  the ground (extra rolling friction on exit). The dribbler's residual spin at the moment of the
  kick isn't free to ignore — it keeps trying to pull the ball back even as the kick pushes it
  forward, so it measurably saps the kick's exit speed unless the control code deliberately backs
  the dribbler off first (partially, not necessarily to zero — that's a client decision, see
  below).
- **What the simulator does NOT need to replicate**: the BLDC motor's electrical model (current,
  back-EMF, torque curve), the capacitor's real RC discharge curve, or PWM/transistor-level
  control. `RobotCommand.dribble_speed` and `RobotCommand.kick_power` are already the abstraction
  boundary — a `[-1, 1]` speed command and a `[0, 1]` power request, exactly like a real HAL layer
  would expose to control code. Model the *macroscopic mechanical effect* (capture force, spin
  transfer, load-based speed sag, impulse + torque, capacitor charge level over time), not the
  circuit that produces it.

## Decisions already made (discussed with the user — don't re-litigate)

1. **Roller↔ball contact is fully hand-rolled, not a real Bullet collision body.** Do not create
   a `btCylinderShape`/`btConeShape` rigid body for the roller. Instead: a geometric "capture
   zone" test (is the ball's position, in the robot's local frame, within the roller's box) plus
   an explicit friction/centering force computed by formula and applied directly to the ball's
   rigid body every frame — the same style `Robot::applyDriveForces` already uses for wheel
   friction (see below). This is a deliberate choice, not a shortcut: this session already hit an
   **unresolved** Bullet solver instability from sustained cylinder-vs-sphere contact (robot
   chassis pushing the ball against a wall produces a slowly growing, spurious yaw — see the git
   log around `9e9a805` and the comment on `m_numIterations = 30` in `physics.cpp`). The dribbler
   needs *continuous, stable* contact for as long as the ball is captured — the one scenario where
   that bug class would hurt most — so don't reintroduce it here.
2. ~~**Kicker impulse goes through Bullet's own `applyImpulse(impulse, relativePosition)`**, not a
   hand-rolled chip/push-down model. An off-center impulse (offset by the configured plunger
   height from the ball's center) naturally produces the right torque for chip/press-down effects
   via Bullet's own rigid body dynamics — don't special-case "if height offset is negative, add
   upward velocity" or similar; let the physics do it.~~ **Superseded — this was wrong.**
   `btRigidBody::applyImpulse(impulse, rel_pos)` splits into `applyCentralImpulse(impulse)`
   (always `Δv = impulse/mass`, independent of `rel_pos`) plus a torque impulse from
   `rel_pos.cross(impulse)`. The offset changes angular velocity (spin) only — it can never change
   the ball's linear/COM velocity, so a pure height-offset impulse literally cannot chip the ball,
   at any offset, no matter the sign. Verification item 6 below (claiming this was tested and
   worked) was never actually rigorous. Reported by the user as "ball doesn't jump when kicked
   below center, though it should" and fixed the same day by adding a real angle term
   (`/robot/kicker/chip_max_angle`) that tilts the impulse vector itself for negative
   `height_offset` — see the long comment above `Kicker::requestKick` in `kicker.cpp`. A flat
   plunger plate genuinely can't chip from contact height alone in reality either (its contact
   normal is horizontal regardless of contact height); real RoboCup chip kickers use a
   mechanically angled plate, which `chip_max_angle` approximates.
3. **The capacitor's charge is actually simulated over time in the simulator**, not just trusted
   from the client. `RobotCommand.kick_power` (0.0–1.0) is a *request*; the impulse actually
   delivered is `min(requested_power, current_charge_fraction) * max_impulse`, and firing drains
   the capacitor, which then recharges toward 1.0 over `/robot/kicker/charge_time` seconds. A
   second kick fired before recharge completes is measurably weaker even if the client requests
   full power. Expose the live charge fraction over gRPC so client code can poll it before firing
   (see proto section) — this is what makes the "measurably weaker" behavior testable/usable
   rather than a surprise.
4. **A small physical "lip" is added to the robot's collision geometry** so a ball resting in the
   capture position doesn't roll away when the dribbler is off (zero commanded speed ⇒ zero
   capture force, but the ball should still physically rest there, e.g. after the client stops
   the roller). This requires turning the robot chassis from a single `btCylinderShape` into a
   `btCompoundShape` (chassis cylinder + a small box "lip" in front) — see Phase 0.
5. **Sharp-turn ejection and kick-drag from residual spin are not scripted.** They must emerge
   from correctly modeling (a) the ball's real angular momentum from roller friction and (b) the
   capture zone being a real geometric region the ball can leave when the robot's rotation moves
   the zone faster than the roller's centering force can keep up. If you find yourself writing a
   special case like `if (turning_fast) eject_ball(...)`, stop — the zone/force model is missing
   something, don't paper over it with a scripted trigger. The testing checklist below has a
   concrete way to verify this emerges rather than being asserted.

## Config additions

Extend the existing `/robot/dribbler` and `/robot/kicker` sections in `simulator/configs/robot.json`
(don't rename `max_power`/`charge_time`/`max_speed`/`radius` casually — they're already there;
redefine `radius` as `radius_center`/`radius_edge` since the roller now needs a taper profile,
and document units precisely since this is the exception-prone part of the codebase):

```jsonc
"kicker": {
    "max_power": 8.0,       // N·s (impulse) delivered to the ball at full charge, full power request
    "charge_time": 0.2,     // seconds to recharge from empty to full
    "height_offset": 0.0,   // mm, plunger height relative to ball center; + = above, - = below
    "range": 40.0            // mm, max ball-to-plunger distance for a kick to register at all
},
"dribbler": {
    "max_speed": 1000.0,          // RPM at dribble_speed = ±1.0
    "radius_center": 8,           // mm, roller radius at its lateral center (z=0)
    "radius_edge": 12,            // mm, roller radius at the ends of its capture zone
    "length": 70,                 // mm, roller length along robot-local Z (capture zone width)
    "forward_offset": 95,         // mm, roller axis position forward of chassis center (local +X)
    "height_offset": 15,          // mm, roller axis height above the ground
    "capture_tolerance_forward": 15, // mm, +/- forward slack around forward_offset counted as "in zone"
    "capture_tolerance_height": 10,  // mm, +/- height slack around height_offset
    "friction": 1.2,              // Coulomb coefficient, roller surface vs ball surface
    "normal_force": 0.6,          // N, effective press force between roller and a captured ball
    "motor_time_constant": 0.03,  // seconds, first-order lag (same pattern as /robot/motor/time_constant)
    "load_sag_gain": 0.4,         // 0..1, fraction of max speed lost when a ball is loading the roller
    "lip_height": 6,              // mm, static bump height added to the chassis collision shape
    "lip_forward_offset": 88,     // mm, bump center forward of chassis center
    "lip_thickness": 10           // mm, bump depth along local X
}
```

Units follow the existing convention exactly (`AGENTS.md`): everything here is millimetres except
`friction` (dimensionless), `normal_force` (already in the ball/robot's kg·mm/s² unit system —
i.e. compute it consistently with how `Robot::applyDriveForces` derives `normal = m_mass *
m_gravity / kOmniWheels` from mass in kg and gravity in m/s², don't introduce a third force unit),
`load_sag_gain` (dimensionless), `motor_time_constant` and `charge_time` (seconds), `max_power`
(N·s). Convert every millimetre value to metres exactly once at load time, in whichever class owns
it — same `MM = 1000.0f` pattern as everywhere else.

## Proto additions

`simulator/proto/simulator.proto` — add to `SensorData` (next free field numbers are 19/20; ball
position used 16-18):

```protobuf
    // Dribbler motor's actual (lagged, load-sagged) speed, RPM, signed same as dribble_speed.
    float dribbler_rpm = 19;

    // Kicker capacitor charge level, 0.0 (empty) to 1.0 (full).
    float capacitor_charge = 20;
```

`RobotCommand.kick_power` / `dribble_speed` field *comments* need updating to document the
semantics precisely now that they're not stubs — `kick_power` is a *request* clamped to available
charge (see Decisions §3); document the sign convention for `dribble_speed`: **positive =
capture direction** (roller surface at the ball contact point moves toward the robot, i.e. the
direction that pulls a ball in and holds it), matching how a positive value would intuitively map
to "dribbling" rather than "ejecting". State this explicitly in the `.proto` comment the way the
`LidarPoint.angle` comment states its own sign convention — don't leave it implicit.

## Implementation phases

### Phase 0 — Robot collision shape becomes a compound

`Robot::m_collisionShape` is currently a single `std::unique_ptr<btCollisionShape>` holding a
`btCylinderShape` (`robot.cpp` around line 73). Change it to a `btCompoundShape` containing:
- the existing chassis cylinder, at local origin (no change to its size/position — this must be
  a no-op for everything that currently depends on chassis geometry, including CCD thresholds
  computed from `radius`);
- a small box for the lip, positioned at `(lip_forward_offset, <some height>, 0)` in the chassis's
  local frame, sized from `lip_height`/`lip_thickness`/the dribbler's `length`.

Keep the constituent shapes alive (`m_chassisShape`, `m_lipShape` members, or a
`std::vector<std::unique_ptr<btCollisionShape>>` like `Physics` already uses for walls) — a
`btCompoundShape` doesn't own its children's memory. `Physics::debugDraw`'s switch on
`shape->getShapeType()` doesn't handle `COMPOUND_SHAPE_PROXYTYPE` — either add that case
(iterate `getNumChildShapes()`/`getChildShape(i)`/`getChildTransform(i)` and recurse into the
existing per-type draw code) or explicitly skip compound children with a comment explaining why,
but don't let debug draw silently stop working for the robot.

**Verify no regression** before moving on: rerun the existing straight-line/strafe/turn tests
referenced in `docs/tasks/omni-wheel-dynamics.md` (a clean forward burst should still show
`dyaw≈0`) — swapping the collision shape type can change how Bullet computes contact response
even though `calculateLocalInertia` and CCD setup should be unaffected as long as you keep using
the chassis radius for those, not the compound's AABB.

### Phase 1 — `Ball` needs a small API surface for external forces

`Ball` currently exposes nothing but `position()`. Add read access to what the dribbler/kicker
force model needs: linear velocity, angular velocity, and a way to apply a force+torque or an
impulse+relative-position. Either expose `btRigidBody* body() const { return m_body.get(); }` and
let `Dribbler`/`Kicker` call Bullet's API directly (simplest, matches how `Physics` already
exposes `world()` to other classes), or add narrow wrapper methods
(`applyDribblerForce(glm::vec3 force, glm::vec3 torque)`, `applyKickImpulse(glm::vec3 impulse,
glm::vec3 relativePos)`, `linearVelocity()`, `angularVelocity()`, `radius()`) if you'd rather keep
Bullet types out of the new classes' public interfaces. Either is fine; match whichever style
feels more consistent with the rest of the codebase once you're in it — the `body()` accessor is
probably less code and this project already leans toward exposing the underlying Bullet handle
(`Physics::world()`) rather than wrapping every operation.

### Phase 2 — `Dribbler` class

New `simulator/src/dribbler.h`/`.cpp`, peer to `LidarSensor`/`Ball` (owned by `App`, not by
`Robot` — it needs both the robot's pose and the ball's state every frame, same shape of
dependency `LidarSensor::update(robotPos, robotYaw, dt)` already has).

`void Dribbler::update(const Robot& robot, Ball& ball, float dt)`:

1. **Commanded speed → lagged actual speed**, same first-order-lag pattern as
   `Robot::applyDriveForces`'s per-wheel `m_wheelSpeed`: `alpha = dt / (motor_time_constant +
   dt); m_actualSpeed = lerp(m_actualSpeed, m_targetSpeed * max_speed, alpha)`, where
   `m_targetSpeed` is the last `dribble_speed` command (`[-1, 1]`).
2. **Capture zone test**: transform the ball's world position into the robot's local frame using
   the exact rotation this codebase already uses everywhere (`AGENTS.md`'s yaw convention —
   copy the `world → local` transform from `Robot::applyDriveForces`'s `linX`/`linZ` derivation,
   don't re-derive the sign by hand again). Ball is "in zone" if `localX` is within
   `forward_offset ± capture_tolerance_forward`, `localY` (ball center height) is within
   `height_offset ± capture_tolerance_height`, and `|localZ| <= length/2`.
3. **If out of zone**: apply nothing, `m_loadFraction = 0` (no sag).
4. **If in zone**: 
   - Effective roller radius at this contact: `r = lerp(radius_center, radius_edge, |localZ| /
     (length/2))`.
   - Roller surface velocity at the contact point, in the robot's local frame: purely tangential,
     magnitude `|m_actualSpeed_rad_per_s| * r`, direction `-sign(m_actualSpeed) * localX_axis`
     (positive `dribble_speed` ⇒ surface moves in local `-X`, toward the robot — matches the
     capture-direction convention from the proto section above).
   - Ball's own surface velocity at that same point (local frame): ball's linear velocity plus
     `ω_ball × (contactPoint - ballCenter)`, both rotated into local frame the same way.
   - Slip velocity = ball surface velocity − roller surface velocity (local frame, project onto
     the local `X`/`Y` plane — the roller doesn't constrain lateral `Z` slip, that's what lets
     the ball self-center along the taper instead of being rigidly pinned in `Z`).
   - Friction force, same clamped-Coulomb pattern as `applyDriveForces`: `desired = ballMass *
     slip / dt` (deadbeat target, no separate response-gain needed here since this isn't fighting
     a coupled rotation/translation mismatch the way the chassis is — but add one if you see
     ringing during testing), then clamp magnitude to `friction * normal_force`. Convert to world
     frame and `applyForce(worldForce, contactPointWorldOffsetFromBallCenter)` on the ball body —
     applying it at the actual contact point (not the ball's center) is what generates the
     backspin torque for free via Bullet's own force/torque coupling; don't separately compute
     and apply a torque.
   - Track `m_loadFraction` (e.g. proportional to the applied force magnitude relative to
     `friction * normal_force`) and feed it back into the motor lag as a speed *cap*: `effective
     target = m_targetSpeed * max_speed * (1 - load_sag_gain * m_loadFraction)`, so a captured
     ball measurably slows the roller.
5. Publish `dribbler_rpm` (the lagged actual speed, converted back to RPM) for `SensorData`.

### Phase 3 — `Kicker` class

New `simulator/src/kicker.h`/`.cpp`, same ownership pattern.

- `m_charge` in `[0, 1]`, recharges every frame: `m_charge = min(1.0, m_charge + dt /
  charge_time)`.
- `void requestKick(float power)` — call only when `kick_power > 0` arrives from a command (mirror
  `grpc_server.cpp`'s existing `if (m_state.kickPower > 0.0f) m_robot->kick(...)` gate). Compute
  `deliveredFraction = min(power, m_charge)`; if the ball isn't within `range` of the kicker point
  (reuse the dribbler's capture-zone forward/height math, or a simpler point-distance check against
  the configured kick origin — your call, but state which you picked and why in a comment), do
  nothing and don't drain the capacitor. Otherwise: `impulseMag = deliveredFraction * max_power`,
  apply `ball.body()->applyImpulse(worldForwardDir * impulseMag, contactOffsetFromBallCenter)`
  where `contactOffsetFromBallCenter` is `(0, height_offset, 0)` rotated into world (or just
  `(0, height_offset, 0)` directly, since it's already relative — don't rotate a pure-Y offset by
  yaw, it's rotation-invariant); then `m_charge -= deliveredFraction` (clamped to ≥ 0).
- Publish `m_charge` for `SensorData.capacitor_charge`.

### Phase 4 — wiring

- Remove `Robot::kick()`/`Robot::dribble()` and their declarations — replaced by `Dribbler`/`Kicker`.
- `App` owns `std::unique_ptr<Dribbler> m_dribbler` and `std::unique_ptr<Kicker> m_kicker`
  (same pattern as `m_lidar`/`m_ball`), constructed/`init()`'d alongside them.
- In `App::update()` (`simulator/src/app.cpp`), **the ordering matters**: `Dribbler::update()`
  must run *before* `m_physics->step(dt)` (it applies a force that only takes effect on the next
  `stepSimulation`, exactly like `Robot::applyDriveForces`), and `Kicker::requestKick()` similarly
  needs to happen before the step it's meant to affect. Insert both calls at line ~312-313,
  immediately around the existing `m_robot->applyDriveForces(dt); m_physics->step(dt);` pair —
  don't put them after `syncFromPhysics()`, that would delay their effect by a full frame.
- `GrpcServer` needs `setDribbler(Dribbler*)`/`setKicker(Kicker*)` setters (mirror
  `setRobot`/`setBall`) and `GrpcServer::update()` needs to call `m_dribbler->setTargetSpeed(...)`
  and `m_kicker->requestKick(...)` where it currently calls the now-removed `m_robot->dribble(...)`
  / `m_robot->kick(...)` — same `if (m_state.kickPower > 0.0f)` gate for the kick call. Wire the
  new pointers in `App::init()` next to the existing `m_grpcServer->setBall(m_ball.get())` line.
- `SharedState`/`SimulatorServiceImpl`/the `SensorData` publishing block in
  `GrpcServer::update()` need `dribblerRpm`/`capacitorCharge` fields added and populated, same
  pattern as `m_state.ballPosition = m_ball->position();`.
- `python/robot_hal.py`'s `kick()`/`dribble()` already send the right proto fields — no change
  needed there unless you want to add getters for the two new `SensorData` fields (optional, but
  do it if you're already touching the file — `get_dribbler_rpm()`/`get_capacitor_charge()`
  mirroring the existing accessor style).

## Testing checklist

Use the same empirical gRPC-driven methodology as the rest of this project (a Python script using
`SimRobotHAL` or raw `simulator_pb2` stubs, not visual inspection) — write throwaway scripts in
your scratchpad, not the repo.

1. **Capture**: place/let the ball roll to just in front of the robot, command `dribble_speed =
   1.0`, wait — ball's position should settle at roughly `forward_offset` from the chassis and
   stay there (not creep away) for several seconds with the robot stationary.
2. **Hold while driving**: with the ball captured, command `vx = 1.0` for ~1s — the ball should
   move with the robot, staying roughly `forward_offset` ahead, not lag behind or get left in place.
3. **Rest without spin**: capture the ball, then command `dribble_speed = 0`. Ball should stay
   resting in place (lip geometry holding it) rather than rolling away — but should *not* still be
   pulled tighter against the robot (zero capture force at zero speed).
4. **Sharp-turn ejection emerges, isn't scripted**: capture the ball, then command a large
   `omega` for a short burst. Verify the ball's trajectory diverges from the capture zone
   smoothly (still has forward momentum + residual spin at exit) rather than teleporting or being
   force-launched by special-case code — if you added an explicit "eject" branch anywhere, that's
   a sign the zone/force model needs fixing instead.
5. **Kick, neutral height**: `height_offset = 0`, captured ball, fire `kick_power = 1.0` at full
   charge — ball's vertical velocity component right after the kick should be small relative to
   its horizontal velocity.
6. **Kick, low plunger**: negative `height_offset` — ball should show a real, positive vertical
   velocity component (chip) that grows toward `chip_max_angle` as `height_offset` approaches
   `-ball_radius`, vs. exactly zero vertical velocity at `height_offset = 0`. (Verified 2026-09-23:
   `height_offset = -10mm`, default `chip_max_angle = 30`, full power — ball reached ~0.14m peak
   height and came back down with a real bounce; `height_offset = 0` stayed flat at exactly
   ball-radius height throughout, confirming the fix didn't change the neutral-height baseline.)
7. **Capacitor drains and recharges**: fire two kicks with less than `charge_time` seconds between
   them (both at `kick_power = 1.0`) — the second kick's resulting ball speed should be measurably
   lower than the first's. Fire a third after waiting a full `charge_time` — should be back to
   full strength.
8. **No regression**: rerun a clean forward/strafe/turn burst (no ball involved) and confirm
   `dyaw`/oscillation behavior matches what `docs/tasks/omni-wheel-dynamics.md` established —
   the Phase 0 compound-shape change is the most likely place to accidentally regress this.

## Force model revision: position + rotation aware capture (supersedes the original design)

The first implementation (reviewed above) computed the roller's target contact-point velocity in
the robot's *local* frame without accounting for the frame itself rotating, and explicitly
discarded the lateral (`Z`) slip component ("the roller doesn't constrain lateral slip"). Two
real problems came from this, found via the same empirical (gRPC-script, not visual) testing
methodology used throughout this project:

1. **No actual centering force existed.** A ball entering the capture zone just stayed wherever
   it crossed the boundary — it was never pulled to the middle of the pocket, contradicting the
   intended design (a real dribbler's tapered roller self-centers the ball).
2. **A ball held stationary by a spinning dribbler, robot not turning at all, would spontaneously
   drift sideways and eventually eject** — reproducible from a fully-settled, perfectly centered
   start, with no lip or approach-driving involvement (both were tested and ruled out). The
   backspin the dribbler imparts and the ball's own real Bullet ground-rolling contact were
   fighting over the same rotational DOF with no restoring force to counteract the imbalance.

Fix: `Dribbler::update` now computes a single **world-frame target velocity for the contact
point**, combining three physically distinct components before one shared Coulomb-clamped
force/torque is derived from the slip:

- **Rigid attachment** (`V_robot + omega_robot × contactOffset`) — the velocity a point fixed to
  the rotating robot would have. This is what makes turning cost something: holding the ball
  through a turn means matching a centripetal/tangential demand that grows with turn rate, and a
  hard reverse is a sudden change in `V_robot` the clamped force may not track — both now
  genuinely eject the ball when the demand exceeds the grip budget, exactly as described by the
  user: *"при повороте появляется нормальное ускорение, из-за которого мяч может не удержаться и
  влететь из лунки — но не значит что он перестаёт вращаться."* Ejected balls keep whatever
  spin/velocity they had (nothing here resets the ball on zone-exit, only the force stops being
  applied), so a released, still-spinning ball chasing a retreating robot (the scenario the user
  described) falls out of the existing physics for free.
- **Centering** — a spring-like pull (NOT a one-timestep deadbeat — see the code comment on
  `m_centeringTimeConstant` for why `posErr / dt` was a bug in its own right, saturating the
  clamp on every frame regardless of how small the error was, leaving no budget for anything
  else) toward the middle of the pocket. This is what makes entering the capture zone at all —
  not where in it — the thing that matters, matching *"если мяч попадает в эту область то по
  факту попадает в лунку и начинает держаться в центре."*
- **Spin** — unchanged: the roller surface's own tangential target.

All three share **one** Coulomb clamp (`friction × normal_force`) — a single grip budget, not
separate allowances, matching *"все закруты дриблера создают некую прижимную силу"*.

A second, unrelated bug surfaced during this same testing round: `/robot/dribbler/lip_forward_offset`
(88mm) put the physical lip *inside* the ball's actively-captured resting position, so real Bullet
contact between them fought the hand-rolled force every frame and leaked a spurious, slowly
growing robot yaw with nothing commanded. Moved to 70mm (clear of the capture equilibrium) —
see the comment in `Robot::init` above the `lipForward` config read.

Current tuned config (`friction: 1.5`, `normal_force: 1.0`, `centering_time_constant: 0.03`):
verified via repeated gRPC test scripts — a captured ball at rest stays in a bounded ~±4mm
oscillation around dead center for 7.5s+ with no growth and no robot yaw drift; a gentle turn
(~1 rad/s) is held throughout; a faster turn (~2 rad/s) or a hard reverse burst reliably ejects
the ball within a few hundred ms while it keeps spinning. The exact turn rate at which grip is
lost is not perfectly crisp run-to-run (small, expected sensitivity in a clamped/saturating
controller near its limit) — treat "~2 rad/s" as an approximate threshold, not a guarantee, if
tuning further.

## Second revision: real pocket recession, and the lip's replacement

Follow-up request: the ball should visibly/physically sit recessed into the front of the robot
by a configurable distance (default 15mm), matching how a real dribbler ball nestles into a
concave pocket rather than resting flush against the chassis.

The straightforward attempt — just moving `forward_offset` closer to the chassis center so the
ball's target position overlaps the chassis by 15mm — reproduced exactly the bug class this doc
already warns about: the chassis is a *real* Bullet collision cylinder, so it physically blocked
the ball from ever reaching the new target, and the hand-rolled centering force fighting that
real contact every frame shoved the ball around unpredictably during approach (not the earlier
"spontaneous eject at rest" bug, but the same underlying cause — a hand-rolled force and real
Bullet contact both trying to own the same space).

Fix: the chassis's *collision* cylinder is now genuinely smaller than its rendered radius, by
`pocket_depth` (`/robot/dribbler/pocket_depth`, default 15mm — see the comment in `Robot::init`).
`forward_offset` is chosen so the captured ball's near surface lands exactly tangent to this
shrunk radius, so it has real solid structure to rest against instead of empty space the
hand-rolled force alone has to hold it in. This also makes the original compound shape's separate
front "lip" child redundant — it existed only to give a resting, unpowered ball something solid
to lean on, which the shrunk chassis now does on its own — so it was removed (`Robot` no longer
has `m_lipShape`; the compound shape currently has one child). `/robot/dribbler/lip_height`,
`lip_forward_offset`, and `lip_thickness` were removed from `robot.json` accordingly.

The pocket's own shape — described as a concave arc, not a full circle, giving "some advantage"
holding the ball through a turn — is modeled as a small bonus to the dribbler's effective grip
budget (`pocket_grip_gain`, N per metre of `pocket_depth`, default 10 → +0.15N / +15% at the
default 15mm), not as real cutout geometry (Bullet compound shapes can't represent a concave
notch without a union of many convex pieces approximating it, which felt like a lot of geometry
for a "small advantage" — see the simplification note in `Robot::init`'s comment). It still shares
the one Coulomb clamp everything else does, per the first revision above.

Known simplification, stated honestly rather than fixed silently: shrinking the whole chassis
cylinder (rather than just a frontal notch/arc) means a ball — or, once multi-robot support
exists, another robot — can in principle approach `pocket_depth` closer to the chassis from *any*
side, not just the front. Harmless today (single robot, no robot-vs-robot collision yet).

**A debugging note for whoever touches this next**: while chasing what first looked like a
turn-ejection regression from this change, an already-present *uncommitted* local edit to
`/robot/dribbler/max_speed` (1000 → 4000 RPM) and `radius_edge` (12 → 14mm) — unrelated to this
task, apparently a manual tuning experiment via the keyboard dribbler control — turned out to be
the actual cause: at 4000 RPM the roller's own spin demand alone saturates enough of the grip
budget that even the *original* `forward_offset`/pocket-free geometry fails a 1 rad/s turn hold.
Confirmed by isolating each changed variable independently (torque cap, pillars/camera
rendering, the lip, `pocket_depth` itself — none of them reproduced it alone) against a `git
stash`ed true baseline, three repeated runs each, before finding the real culprit. Left that
`max_speed`/`radius_edge` edit as found (not this task's to revert) — but if dribbler behavior
looks wrong again, check `robot.json` against `git diff` before assuming new code broke it.

## Third revision: cradle plate (normal) vs. roller (tangential) — separate grip budgets

Follow-up request: model the physical plate more literally. Real hardware holds the ball with
two distinct contacts below the roller — a concave plate machined to the ball's own radius (~10mm
tall, bottom edge ~4mm off the ground, so its center of curvature coincides with the ball's own
seated center) plus the roller itself just above it. Because the plate matches the ball's
curvature, whatever force it exerts on the ball is necessarily perpendicular to that shared
spherical surface at the contact point — described by the user as *"при поворотах на мяч силы
действуют перпендикулярно поверхности сферы которую она описывает, так при поворотах мяч не
дергает и держится не только из-за дриблера"* — and the ball should eject when *"резко
повернуться и нормальное ускорение станет больше чем сила которую дает дриблер для удержания
мяча."*

The second revision's model already computed a single **world-frame target velocity** for the
contact point (rigid-attachment + centering + spin) and derived **one** combined-direction,
Coulomb-clamped force from the slip against it — explicitly "no separate lateral allowance" (see
the first revision above). That single combined direction is the problem the user is describing:
whatever direction zeroes the slip fastest could include a large chunk of the roller's own huge
along-roller-axis spin demand (`actualSpeed * r`, easily 1+ m/s) mixed in with the turning-hold
demand, so the two fought each other inside one clamp — this is what produced the "ejects, then
flies out sideways on re-capture as if spun very fast" symptom reported 2026-09-24.

Fix: split the slip into two **axes**, each with its own Coulomb budget, instead of one combined
vector with one budget:

- **Roller axis** — along the ball's own local-position direction (`localX, localZ`, normalized).
  This is where the roller's own target (`spinWorld`, always along local `X`) lands, so it's the
  roller's job: pull the ball in and impart backspin. Budget: `friction * normal_force` (plain
  roller-vs-ball surface friction, no pocket bonus — the pocket doesn't add extra grip in the
  direction the roller spins).
- **Cradle axis** — perpendicular to the roller axis (rotated 90°). This is where the
  rigid-attachment term's rotational component lands (basic circular motion: a point rigidly
  attached to a rotating body has velocity *tangential* to its offset from the rotation axis, i.e.
  perpendicular to the ball's own local-position vector, not parallel to it — verified empirically,
  see the note below) and where the centering spring's lateral correction lands. Budget:
  `friction * (normal_force + pocket_depth * pocket_grip_gain)` — the same total the old model's
  single clamp used, but now undiluted by the roller's own spin demand. This is the "сила, которую
  даёт дриблер для удержания" — exceed it and the ball drifts off its seated position and
  eventually leaves the capture zone, i.e. ejects, exactly matching the user's description.

**A mistake caught by testing, not code review, worth recording**: the first attempt assigned the
axes the other way around — "normal/cradle" = *along* the ball's own local-position vector,
reasoning that a turn's centripetal demand points from the ball toward the robot's rotation axis
(true for *acceleration*). But the model works in terms of a **target velocity**, and velocity
under pure rotation is tangential to the offset, not radial — so the turning demand actually lands
on the axis *perpendicular* to the ball's position, and the roller's own spin (along local `X`, the
ball's position direction when centered) is what's parallel to it. The first (wrong) assignment
measurably regressed gentle-turn holding versus the pre-existing shared-clamp model (0.176m max
lateral drift during a scripted 1 rad/s turn vs. the old model's 0.073m on the same test); swapping
the two axes fixed it (0.038m — better than the old model, not worse). Re-verify empirically
(gRPC test script, not code review) if you touch this axis assignment again.

Verified via a gRPC test script (throwaway, not committed) sweeping turn rate after a clean
capture + 1.5s settle: 0.2/0.5/1.0 rad/s all held with 2-7mm max drift and re-settled near dead
center once the turn stopped; 1.5 rad/s ejected (drift kept growing after the turn stopped instead
of recovering) — consistent with the first revision's own "~2 rad/s, not perfectly crisp run to
run" caveat, which still applies here (a longer sustained 1.0 rad/s turn in a separate run drifted
further, ~38mm, before recovering — the exact boundary is a marginally-stable regime, not a hard
threshold, same caveat as before, not a new one this revision introduced).

`m_loadFraction` (motor sag feedback) now comes from the roller axis's own utilization only, not
the combined force — the cradle/plate force is pure geometry and doesn't load the roller motor.

## Docs to update when done

- `AGENTS.md`: add `Dribbler`/`Kicker` to the class-responsibilities list (same style as the
  existing `LidarSensor`/`Ball` entries).
- `ROADMAP.md`: mark the dribbler/kicker item done, and note anything you deliberately left as a
  known limitation (following this session's precedent of being explicit about that rather than
  silent — see the commit message on `9e9a805` for the tone/format to match).
