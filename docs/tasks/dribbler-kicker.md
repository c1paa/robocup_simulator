# Dribbler + kicker mechanics

Implements the two remaining "field player" mechanisms: the spinning dribbler roller that
captures and holds the ball against the robot, and the solenoid kicker that fires it. Both are
currently no-op stubs (`Robot::kick()` / `Robot::dribble()` in `simulator/src/robot.cpp` just do
`(void)power;`), but the gRPC transport for them already exists end-to-end (`kick_power` /
`dribble_speed` in `RobotCommand`, wired through `SharedState` → `GrpcServer::update()` →
`Robot::kick`/`dribble`) and `robot.json` already has placeholder config sections
(`/robot/kicker`, `/robot/dribbler`) — this task fills in the physics behind them, not the
transport.

**Status: implemented and reviewed, with one known unresolved limitation** — see
"Known limitation" near the end of this doc before tuning `/robot/dribbler/friction` or
`normal_force` back up. Read that section first if a captured ball is drifting/ejecting sideways
with no turn commanded.

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
2. **Kicker impulse goes through Bullet's own `applyImpulse(impulse, relativePosition)`**, not a
   hand-rolled chip/push-down model. An off-center impulse (offset by the configured plunger
   height from the ball's center) naturally produces the right torque for chip/press-down effects
   via Bullet's own rigid body dynamics — don't special-case "if height offset is negative, add
   upward velocity" or similar; let the physics do it.
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
6. **Kick, low plunger**: negative `height_offset` — ball should show a measurably larger upward
   velocity component (chip) than the neutral case, same power.
7. **Capacitor drains and recharges**: fire two kicks with less than `charge_time` seconds between
   them (both at `kick_power = 1.0`) — the second kick's resulting ball speed should be measurably
   lower than the first's. Fire a third after waiting a full `charge_time` — should be back to
   full strength.
8. **No regression**: rerun a clean forward/strafe/turn burst (no ball involved) and confirm
   `dyaw`/oscillation behavior matches what `docs/tasks/omni-wheel-dynamics.md` established —
   the Phase 0 compound-shape change is the most likely place to accidentally regress this.

## Known limitation (found during review, not fixed)

Empirically verified (repeatedly, via gRPC test scripts, not visual inspection): a ball held
stationary in the capture zone by a spinning dribbler — robot not turning, `omega = 0` the whole
time — starts drifting **laterally** (along local `Z`, out the side of the zone) after roughly
0.7-1.3s of continuous capture, and the drift accelerates until the ball leaves the zone
entirely. This reproduces from a fully-settled, perfectly centered start (no approach driving,
no lip contact involved — both were tested and ruled out as the cause), and its rate scales
with `/robot/dribbler/friction` × `normal_force`: at the doc's original suggested defaults
(1.2 × 0.6 N) it blows up in under 2s; at gentler values (0.6 × 0.15 N — the values now shipped
in `robot.json`) it takes several seconds, long enough for a realistic approach→capture→kick
sequence, but it does not go away.

Root cause, as far as this review got: the dribbler intentionally imparts **backspin** (positive
local-`Z` angular velocity) so a captured ball's own weight/ground-friction pulls it toward the
robot. But the ball is *simultaneously* in real Bullet contact with the ground, which enforces
its own rolling-without-slip relationship between forward velocity and `Z`-axis spin — and for a
ball being held roughly stationary while backspun, that ground relationship wants the *opposite*
sign of `Z`-spin from what the dribbler is imposing. The two constraints (one hand-rolled, one
real Bullet contact) are fighting over the same rotational DOF and can't both be satisfied, so
the dribbler force rarely reaches zero even once the ball looks "settled" — it keeps injecting
momentum every frame. That's consistent with everything observed: why lowering
`friction`/`normal_force` (weaker injection) delays but doesn't prevent it, why a
`response_gain` damping term (added during this review, mirroring
`Robot::applyDriveForces`'s `m_frictionResponseGain`) helped smooth per-frame chatter but didn't
fix the underlying drift, and why the eventual ejection direction is a genuine sideways slide
(not a symmetric wobble) — some small seed asymmetry (most likely floating-point noise at the
ball-ground contact) gets steadily amplified by the unresolved conflict rather than damped out.

This is the same *class* of bug as the sustained-contact chassis-vs-ball instability documented
in the git history around commit `9e9a805` (continuous force against a body already in another
contact relationship, resolved by two different mechanisms that don't agree) — not a copy-paste
of that bug, but the same underlying weak point in this project's hybrid
hand-rolled-force-plus-real-Bullet-contact approach. A real fix likely needs either (a) replacing
the ground's real rolling-friction contact with a hand-rolled ground model too whenever a ball is
dribbler-captured (consistent modeling instead of two independent ones), or (b) deriving the
dribbler's target spin from the ball's actual rolling state instead of a fixed roller-surface
target. Both are bigger changes than this review's scope. Left as a follow-up, the same way the
chassis-vs-wall instability was — don't try to "tune it away" further with friction/normal_force
without addressing the underlying conflict, per the lesson already learned from that earlier bug.

Practical impact today: capture-and-hold-briefly-then-kick (the realistic gameplay sequence)
works reliably within a few seconds; a robot that dribbles in place for a long time without
acting will eventually lose the ball sideways with no visible cause. If this becomes a real
problem in practice, that's the place to resume investigating, ideally with the same
per-wheel/per-contact force instrumentation approach recommended (and not yet done) for the
chassis-vs-wall case.

## Docs to update when done

- `AGENTS.md`: add `Dribbler`/`Kicker` to the class-responsibilities list (same style as the
  existing `LidarSensor`/`Ball` entries).
- `ROADMAP.md`: mark the dribbler/kicker item done, and note anything you deliberately left as a
  known limitation (following this session's precedent of being explicit about that rather than
  silent — see the commit message on `9e9a805` for the tone/format to match).
