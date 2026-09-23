# Task: LD06-like 2D LiDAR sensor + Python hardware-abstraction seed

This is a self-contained implementation prompt for an AI coding agent (or a human) working on
this repo. It adds a simulated 360° 2D scanning lidar to the robot, matched to the real
**LDROBOT LD06** sensor's information content and imperfections (not its UART wire protocol —
see "What NOT to replicate" below), streams it to clients over the existing gRPC connection,
and seeds a small Python hardware-abstraction layer (`SimRobotHAL`) that will later grow a
`HardwareRobotHAL` counterpart once real Raspberry Pi hardware exists.

Read [`AGENTS.md`](../../AGENTS.md) and [`ROADMAP.md`](../../ROADMAP.md) first. This implements
roadmap item 8. It also requires a small slice of item 7 (field boundary wall collision) as a
dependency — scoped narrowly here, see Phase 1.

## Scope

**In scope:**
- Static Bullet collision boxes for the four field boundary walls only (raycast target for the
  lidar; not goal structure, not full physics response for the ball/robot bouncing off them —
  that remains item 7).
- A `LidarSensor` (or similar) component that raycasts a full 360° sweep against the Bullet
  world each scan, with LD06-matched range, angular density, scan rate, and noise/dropout
  behavior.
- Proto/gRPC changes to stream lidar points to clients.
- Robot-side mounting position/config for the lidar.
- `python/robot_hal.py`: a `SimRobotHAL` class wrapping *all* existing sensor reads and command
  sends (camera frame, pose, odometry, velocity command, kick, dribble) plus the new lidar scan
  accessor, as one coherent interface — this is the seed of the hardware-abstraction layer
  described in `AGENTS.md`'s eventual robot-control code, not the robot-control code itself.

**Out of scope — do not touch, even if it seems related:**
- Full field collision (goal structure, ball/robot physical response bouncing off walls) —
  that's the rest of roadmap item 7. This task only needs walls to exist as raycast targets.
- Ball, kicker/dribbler effects, multi-robot support — separate roadmap items (3, 5, 6). The
  lidar raycasts against the whole Bullet dynamics world generically, so it will automatically
  "see" a ball or other robots once those exist — don't special-case them or add placeholders.
- `HardwareRobotHAL` (the real-Raspberry-Pi backend) — there's no hardware to test against yet.
  `robot_hal.py` should be structured so adding it later is a matter of writing a second class
  with the same method signatures, but don't build a stub for it now.
- The `robot_code/` strategy/vision application itself (state machine, path planning, etc.) —
  future work once the HAL exists and there's something to build against it.
- Replicating the LD06's actual UART packet format (47-byte packets, header `0x54`/`0x2C`,
  CRC-8 poly `0x4D`, 230400 baud) — that protocol only matters for a *real* serial driver
  talking to *real* hardware on a Pi. Inside this simulator, sensor data already rides over
  gRPC; see "What NOT to replicate" below.

## Real LD06 specs (confirmed against manufacturer datasheet pages + a hands-on
reverse-engineering writeup, not from memory)

- **Technology**: direct time-of-flight (ToF), infrared laser, FDA Class I (eye-safe).
- **Range**: rated up to 12 m; practical minimum ~0.02 m (very close returns are unreliable on
  real hardware — treat sub-2cm as no-return).
- **Accuracy**: manufacturer-rated ±15 mm; community hands-on testing shows closer to ±10 mm
  mean error in the 0.03–0.5 m band, degrading at longer range, at grazing incidence angles,
  and on low-reflectivity/glossy/black surfaces (a real ToF lidar can miss returns entirely on
  those — this matters directly for a field with black boundary walls).
- **Sample rate**: fixed 4500 points/second (the ASIC's raw measurement rate).
- **Scan rate**: configurable 5–13 Hz via motor speed, default **10 Hz**.
- **Effective angular density**: sample_rate / scan_rate ≈ 450 points per full 360° rotation at
  the default 10 Hz (real-world measurement puts it at ~480; use ~450–480 as the target, it's
  not a hard-locked number since real angular spacing is irregular — evenly-spaced synthetic
  points are a fine simplification here, see below).
- **FOV**: full 360°, no blind sector (assuming an unobstructed mount).
- **Output per point** (conceptually, not the literal wire encoding): angle, distance,
  intensity/confidence.

## What NOT to replicate

Do not implement the LD06's actual UART packet framing, CRC, or baud rate inside the simulator.
That protocol exists because real hardware talks to a real UART. This simulator's client
interface is gRPC (see `simulator/proto/simulator.proto`), and it should stay that way — the
lidar data just needs to arrive as structured `(angle, distance, intensity)` tuples over the
existing stream. A *future* `HardwareRobotHAL` on a real Pi would be the place that actually
parses 47-byte UART packets from a physical LD06; it has nothing to do with this simulator's
wire format.

## Decisions already made (don't re-litigate these)

1. **Raycasting approach**: use Bullet's ray-test (`btCollisionWorld::ClosestRayResultCallback`,
   one ray per angular sample) against the existing `btDiscreteDynamicsWorld`, not a separate
   geometry/BVH system. This is consistent with how the rest of the sim already leans on Bullet
   for both physics and now sensing.
2. **Points per scan**: fixed, config-driven count (default 450, see config below), evenly
   angularly spaced. This is a deliberate simplification vs. the real sensor's slightly-irregular
   per-packet angular spacing — not worth reproducing, it doesn't change what a consumer of the
   data has to cope with.
3. **Scan rate vs. stream rate**: the lidar's internal scan state updates at its own configured
   rate (default 10 Hz — independent of and likely different from the render/physics tick rate
   and from the camera's `stream_fps`). Each `SensorStream` tick sends the *latest completed
   scan* (same pattern already used for the camera image and odometry — everything bundled into
   one `SensorData` message per tick, not a second RPC). Don't add a second streaming RPC for
   this.
4. **Noise model**: per-point Gaussian noise on distance, std dev from config (default matching
   the ±15mm spec, e.g. `0.015` m, can grow slightly with range if you want — keep it simple,
   a flat std dev is an acceptable first pass), clipped to `[min_range, max_range]`. Points that
   don't hit anything within `max_range`, or that fall inside `min_range`, are **omitted from
   the scan** (not sent as zero/garbage) — mirrors how a real ToF sensor reports "no valid
   return" rather than a fabricated distance.
5. **Dropout model**: beyond distance noise, add a small configurable probability
   (`dropout_probability`, default e.g. `0.01`) of a point being randomly omitted even on a
   valid hit — approximates real-world missed returns on bad-incidence-angle or low-reflectivity
   surfaces without needing per-material reflectivity modeling (out of scope — don't build a
   materials system for this).
6. **Mount position**: lidar is mounted **above** the robot's mirror/dome, at
   `robot.lidar.height` (config, mm, default something like the robot height + 20mm so the
   robot's own body never self-occludes the horizontal sweep). Rays originate from the robot's
   current world position at that height, in the horizontal plane (`+Y` in world space is up,
   per the existing convention — see `AGENTS.md`), rotating with the robot's yaw only insofar
   as angle 0 should be defined in the robot's own body frame (consistent with how the camera
   and drive commands are already robot-frame-relative) — i.e. bake the robot's current yaw
   into each ray's world direction the same way `applyDriveForces`/`Camera` already convert
   local→world.
7. **Field boundary walls get real collision boxes now** (the item-7 dependency): four static
   `btBoxShape`s (or one compound shape) sized from the existing `field.length` /
   `field.width` / `field.wall_height` / `field.wall_thickness` config values already used by
   `Field::render` — reuse those values, don't invent new ones. Static bodies
   (`btRigidBody` with zero mass), added to the same `btDiscreteDynamicsWorld` the robot lives
   in. This deliberately does **not** give the robot itself collision response against these
   walls yet (robot driving through them is a known, tracked item-7 gap) — it only makes them
   visible to raycasts. Don't be tempted to also wire up robot-vs-wall collision response as a
   "quick bonus"; that's a scope decision for whoever picks up the rest of item 7.

## Config additions (`simulator/configs/robot.json`)

Add a `robot.lidar` block, following the existing units convention (config is always mm, deg
where the rest of the file uses deg — convert once at load time in the owning class):

```json
"lidar": {
    "height": 170,
    "min_range": 20,
    "max_range": 12000,
    "points_per_scan": 450,
    "scan_frequency": 10.0,
    "range_noise_std": 15,
    "dropout_probability": 0.01
}
```

(`height`/`min_range`/`max_range`/`range_noise_std` in mm, converted to meters at load time
exactly like every other geometric value in this codebase — see `AGENTS.md`'s units
convention.)

## Proto changes (`simulator/proto/simulator.proto`)

Add a `LidarPoint` message and a `repeated` field on `SensorData`:

```proto
message LidarPoint {
    float angle = 1;     // radians, robot body frame, 0 = forward (+X)
    float distance = 2;  // meters
    float intensity = 3; // 0.0–1.0, synthetic confidence value
}

message SensorData {
    // ... existing fields unchanged ...
    repeated LidarPoint lidar_points = 15;
}
```

Regenerate the Python stubs the same way the existing ones were generated (see
`python/generate_proto.sh`) — don't hand-edit `python/generated/`.

## Implementation phases

**Phase 1 — field boundary collision.** Add the four static wall collision boxes to `Physics`
(or wherever `Robot`'s own rigid body is created — match the existing pattern) at init time,
sized from `field.*` config. Verify with a raycast test (temporary script is fine, doesn't need
to ship) that a ray from robot height, pointed at a wall, actually returns a hit at roughly the
expected distance before moving on — don't build Phase 2 on top of an unverified Phase 1.

**Phase 2 — `LidarSensor` raycasting.** New class (or a method on `Robot`, your call, but if it
grows past ~50 lines of raycasting logic it probably deserves its own `.h`/`.cpp` pair per the
"one class per file" convention in `AGENTS.md`). Each internal scan tick (paced by
`scan_frequency`, independent of the render loop's dt): for `points_per_scan` evenly-spaced
angles around 360°, build a world-space ray from the mount position (robot position + lidar
height) in the direction `(cos(bodyAngle), 0, ...)` rotated into world space using the same
local→world yaw convention as everywhere else in this codebase (`AGENTS.md`'s yaw section —
verify empirically, don't trust a written sign description), run `rayTest` against the dynamics
world, apply the noise/dropout model from decision 4–5, and store the resulting point list as
"latest scan."

**Phase 3 — gRPC wiring.** Populate `lidar_points` in `SimulatorServiceImpl`'s `SensorStream`
handler from the latest scan, through the same `SharedState` bridge pattern already used for
camera/pose/odometry. Confirm with a quick Python script (same style as the empirical tests
used for the omni-wheel work — write one to your scratchpad, don't commit it) that: (a) a scan
against a wall directly ahead returns roughly the expected distance and angle, (b) rotating the
robot 90° rotates the lidar's `angle=0` returns accordingly, (c) point count and rate roughly
match config.

**Phase 4 — `python/robot_hal.py`.** A `SimRobotHAL` class (plain class is fine, don't add an
`abc.ABC` base for a single implementation — YAGNI until `HardwareRobotHAL` actually exists)
wrapping the gRPC stub calls already exhibited in `python/viewer.py`: methods for reading the
latest camera frame, pose, odometry, and lidar scan, and for sending a velocity/kick/dribble
command. Keep method names hardware-agnostic (`get_lidar_scan()`, not `read_uart_lidar()`) since
the same names need to make sense against a real sensor later. This file replaces nothing in
`python/viewer.py` — leave that alone, it's a separate demo client.

## Testing

Empirical, gRPC-driven testing (per this repo's established approach — see the omni-wheel task
history) is more reliable here than manual geometry review, same as it was for the drivetrain
math. At minimum verify: a scan run with the robot centered on the field returns wall hits at
plausible ranges in all directions; rotating the robot rotates the scan pattern correspondingly
in the robot's own frame while the world-space hit points stay fixed; noise std and dropout
rate roughly match config over a few hundred points; the in-simulator app doesn't slow down
(450 raycasts at up to 10Hz is cheap, but confirm it isn't accidentally running every physics
substep).

## After implementing

Update `ROADMAP.md` (mark item 8 done, matching the `~~...~~` pattern used for items 1, 2, 4)
and `AGENTS.md` (add `LidarSensor` — or wherever the logic landed — to the class list, and note
`python/robot_hal.py` in the project layout section) the same way prior task docs describe their
own landing.
