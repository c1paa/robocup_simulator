# Task: catadioptric mirror-camera vision pipeline

This is a self-contained implementation prompt for an AI coding agent (or a human) working on
this repo. It covers the simulator's core feature: a robot-mounted camera looking up into a
curved mirror, producing a real rendered image, streamed to an external Python/OpenCV client
over gRPC.

Read [`AGENTS.md`](../../AGENTS.md) and [`ROADMAP.md`](../../ROADMAP.md) first — this task
implements roadmap items 1 ("gRPC server") and 2 ("real mirror-camera rendering") together,
because the gRPC server has nothing useful to stream until the camera produces a real image.

## Scope

**In scope:** everything needed to go from "robot exists on a field" to "a Python script can
`cv2.imshow()` a live, geometrically correct, mirror-distorted view from the robot's camera,"
plus a debug preview inside the simulator window.

**Out of scope — do not touch, even if it seems related:**
- Ball, physics rigid-body for the robot, kicker/dribbler effects, field collision, multi-robot
  support. These are separate `ROADMAP.md` items. The camera should render whatever is in the
  scene (currently: field + one robot) without assuming a ball exists.
- Anything about how the robot is *driven* other than receiving commands over the already
  existing `SendCommand` RPC and forwarding them to `Robot::setWheelVelocities/kick/dribble`
  (which stay no-ops for kick/dribble — that's the kicker/dribbler roadmap item, not this one).

If a change outside this scope turns out to be required, stop and flag it instead of expanding
scope silently.

## Decisions already made (don't re-litigate these — they were chosen deliberately)

1. **Rendering technique: cubemap + baked direction LUT**, not per-pixel raytracing and not a
   fake procedural warp. Rationale: real-time, physically motivated, and generalizes cleanly to
   any mirror profile without shader-side root-finding.
2. **Mirror shapes: `cone` and `hyperbola` now, architected for more later.** Profile is a
   function `r = f(h)` (radius as a function of height above the mirror's effective viewpoint,
   measured along the optical axis), selected by a `type` field, not hardcoded to one shape.
3. **Delivery to Python: the existing gRPC `SensorStream`/`SendCommand`** from
   `simulator/proto/simulator.proto`. Don't invent a second transport.
4. **In-simulator preview:** a small textured overlay quad in a corner of the existing SDL/GL
   window (not a separate OS window), toggleable with a key.
5. **LUT is baked once in robot-local space (yaw = 0), not per frame.** The cubemap is captured
   every frame in world-axis-aligned orientation from the robot's current world position (only
   translation follows the robot, the six faces never rotate). The compositing shader rotates
   each LUT direction by the robot's *current* yaw (a single 2D rotation of the x/z components)
   before sampling the cubemap. This is what makes the technique fast — do not bake the LUT
   every frame, and do not rotate the cubemap itself.
6. **Effective viewpoint:** for `hyperbola`, use the mirror's real focus (the one that gives the
   single-viewpoint property — compute from `a`/`b`: `c = sqrt(a² + b²)`, focus offset `c` along
   the axis from the mirror's center). For `cone`, there is no true single viewpoint (known
   limitation of conical mirrors); approximate by using the physical camera position as the
   viewpoint. This is a standard, deliberate simplification — don't try to "fix" the cone case
   into something more exact, it isn't necessary for vision-algorithm testing.

## Current inconsistency to resolve first (Phase 0)

Mirror geometry is currently defined in **two disconnected places** that don't agree:

- `robot.json` → `/robot/mirror/{base_height,cone_height,base_diameter}` (mm), read by
  `Robot::init` and used only to size the cone mesh drawn in `Robot::render`.
- `Camera::Mirror` (`camera.h`) has `a`/`b`/`radius` hyperbola-style fields, loaded from
  `/camera/mirror_a`, `/camera/mirror_b`, `/camera/mirror_radius` — keys that **don't exist**
  in `robot.json`, so `Camera` always silently falls back to its hardcoded defaults and never
  reflects what's actually drawn.

Fix this before writing any optics code: there must be exactly one source of truth for mirror
geometry.

## Implementation plan

Work through these phases in order; each should leave the project building and running. Commit
after each phase (see `CONTRIBUTING.md`).

### Phase 0 — unify mirror config and add a `MirrorProfile` type

- New `simulator/src/mirror_profile.h` / `.cpp`. A `MirrorProfile` class/struct that:
  - Has a `Type { Cone, Hyperbola }` and the parameters for whichever type is active.
  - Loads itself from config via a single base path, e.g.
    `MirrorProfile::loadFromConfig(Config& cfg, const std::string& base = "/robot/mirror")`.
  - Exposes `float radiusAt(float height) const` — the profile function `r = f(h)`, `height`
    measured from the mirror's base (matching how `base_height`/`cone_height` already work in
    `robot.cpp`).
  - Exposes `glm::vec3 normalAt(float height, float radiusOnSurface) const` (analytic for both
    cone and hyperbola — both have closed-form slopes; don't use finite differences for these
    two, but keep the function signature generic so a future profile type could).
  - Exposes a **generic** `bool intersectRay(const glm::vec3& origin, const glm::vec3& dir,
    float& tHit) const` that numerically finds where a ray crosses the surface of revolution
    defined by `radiusAt` (e.g. bisection/Newton on `g(t) = radialDistance(origin + t*dir) -
    radiusAt(height(origin + t*dir))` over the valid height range of the mirror). Implement
    this once, generically, in terms of `radiusAt` — do not special-case cone vs. hyperbola
    intersection math separately; that's the point of the abstraction.
  - Exposes `glm::vec3 effectiveViewpointLocal(float cameraHeight) const` per the rule above.
- Update `robot.json`: replace the mismatched `/robot/mirror/*` and `/camera/mirror_*` blocks
  with one consistent block under `/robot/mirror`:
  ```json
  "mirror": {
      "type": "cone",
      "base_height": 150,
      "base_diameter": 170,
      "cone_height": 40,
      "hyperbola_a": 35,
      "hyperbola_b": 40
  }
  ```
  (`cone_height` used only when `type` is `"cone"`, `hyperbola_a`/`hyperbola_b` only when
  `"hyperbola"` — keep both present in the file so switching `type` doesn't require adding
  keys.) Remove `mirror_a`/`mirror_b`/`mirror_radius` from wherever `Camera` was reading them.
- `Robot` owns a `MirrorProfile` (loaded in `Robot::init`) and exposes it via
  `const MirrorProfile& mirrorProfile() const` plus the existing `cameraHeight()`-style getter
  (add one if it doesn't exist — check `robot.h`, `m_cameraHeight` is currently private).
- `Robot::render`'s mirror mesh: for `cone`, unchanged (`drawCone` with the profile's
  base/height). For `hyperbola`, drawing an exact hyperboloid mesh is **not required** for this
  task — draw the same bounding cone shape as a visual placeholder (note this explicitly as a
  known simplification, don't spend time on a real revolved mesh here).

**Acceptance:** project builds; changing `type` in `robot.json` between `"cone"` and
`"hyperbola"` doesn't crash and changes which parameters are read.

### Phase 1 — cubemap capture

- `Camera` gains an offscreen cubemap render target (6 faces, square resolution, new config
  key `/camera/cubemap_resolution`, default e.g. `256`) separate from its existing final-image
  `m_fbo`/`m_renderTexture` (which stays at `/camera/width` x `/camera/height` for the
  composited output).
- Add a method (naming up to you, e.g. `Camera::captureCubemap`) that, given the *scene* to
  draw (field + robot, passed in however fits the existing `App`/`Renderer` structure — e.g. a
  small `std::function<void(Renderer&)>` callback from `App`, or pass references directly) and
  the effective viewpoint in **world space** (robot position + local effective viewpoint
  rotated by robot yaw), renders the 6 cube faces with 90° FOV perspective projections looking
  along ±X/±Y/±Z, always **world-axis-aligned** (never rotated by robot yaw — see decision #5).
- When rendering into the cubemap, skip drawing the mirror mesh itself and the little camera
  indicator sphere (they'd occlude the view from just below/inside them). It's fine — and
  arguably more realistic — to still render the rest of the robot's own body if it's visible
  from the viewpoint; don't spend extra effort special-casing that either way, just don't let
  the mirror/camera markers block everything.

**Acceptance:** you can dump one cubemap face to an image file (temporary debug code, removed
before committing) and see the scene from the robot's position.

### Phase 2 — bake the direction LUT

- On `Camera::init` (and whenever the relevant config changes at runtime, if config hot-reload
  exists — if not, init-time only is fine), for every pixel `(u, v)` of the final `width` x
  `height` output:
  - Compute the camera ray in **robot-local space** from the physical camera position
    (`cameraHeight` above the robot, looking straight up along local +Y) using the configured
    `/camera/fov`.
  - Intersect it with the `MirrorProfile` via `intersectRay`.
  - If no hit: mark this pixel invalid.
  - If hit: compute the surface normal via `normalAt`, reflect the incoming ray direction
    around it, and store the **reflected ray direction in robot-local space**.
- Upload the result as a texture (e.g. `RGBA32F`, rgb = direction, alpha = 1 valid / 0 invalid)
  sized `width` x `height`. This bake only needs to happen once per config, not per frame —
  cache it and skip recomputation if geometry/camera params haven't changed.

**Acceptance:** for `type: "cone"` with a wide FOV, the valid region of the LUT should be
roughly circular (mirror silhouette as seen by the upward-looking camera) — worth a quick
manual sanity check (e.g. dump the alpha channel as a grayscale image) before moving on.

### Phase 3 — composite the final image

- Fragment shader (GLSL — put it in `simulator/shaders/` as real files and load them from disk
  if this pushes shader code meaningfully past what's already inline in `renderer.cpp`; a small
  addition can stay inline for consistency with the existing pattern — your call, just be
  consistent) that, per output pixel:
  1. Samples the direction LUT.
  2. If invalid, outputs a configurable background color (e.g. black) and stops.
  3. Rotates the local direction by the robot's current world yaw (2D rotation of x/z) — passed
     in as a uniform, computed once per frame on the CPU side, not per pixel.
  4. Samples the cubemap with the rotated world-space direction.
  5. Writes that as the output color.
- This replaces the synthetic noise pattern currently in `Camera::update()`. Keep the existing
  Gaussian pixel-noise (`/camera/noise_std`, `/camera/pixel_noise`) as a post-process step
  applied after compositing, so the noise model that's already there and configured keeps
  working (real cameras are noisy; don't remove that on the way to making the image "real").
- `Camera::renderView(robotPos, robotYaw)` becomes: capture cubemap at
  `robotPos + rotate(effectiveViewpointLocal, robotYaw)` → run the compositing pass → read back
  into `m_imageData` (same as today, `imageData()`/`imageWidth()`/`imageHeight()` keep their
  current meaning so `GrpcServer` doesn't need to know any of this happened).
- Add a config knob for how often this actually re-renders relative to the main loop, e.g.
  `/camera/stream_fps` (default `30`) — `App` (or `Camera` itself, tracking elapsed time) skips
  the capture+composite work on frames where the interval hasn't elapsed yet, and just reuses
  the last image. The main SDL/GL loop keeps running at full rate regardless.

**Acceptance:** running the simulator with the in-app preview (Phase 4) shows a recognizable,
radially distorted view of the field through the mirror, that changes correctly as the robot's
position/orientation changes (sanity check: drive the robot in a circle and confirm the
reflected scene rotates the opposite way, since it's an upward-looking mirror).

### Phase 4 — in-simulator preview overlay

- In `App`, add a toggle key (pick one not already bound — check `handleKeyboardInput`,
  currently uses WASD/Shift/Esc — `C` for "camera" is free) that shows/hides a small textured
  quad in a screen corner (e.g. top-left, ~200x150px in screen space) displaying
  `m_camera->imageData()` as a texture, drawn in an orthographic pass after the main 3D
  `Renderer::endFrame()`.
- Doesn't need to be fancy — a flat quad with the camera's `m_renderTexture` (or a texture
  updated from `imageData()`, whichever is more direct given how Phase 3 ended up wiring
  things) bound is enough.

**Acceptance:** pressing the toggle key shows/hides the live mirror-camera view over the main
3D view, updating as the robot moves.

### Phase 5 — real gRPC server

- Rewrite `GrpcServer`/`grpc_server.cpp` to actually run a `grpc::Server`. Threading model:
  - `GrpcServer::start()` spawns a background thread that builds and runs the server
    (`grpc::ServerBuilder`, listening on a configurable port — add `/network/grpc_port` to
    `project.json`, default `50051`).
  - Implement `robocup::Simulator::Service` (from the generated `simulator.grpc.pb.h`) in a new
    small class (e.g. `simulator/src/simulator_service.h`/`.cpp`).
  - `SendCommand` (client-streaming: `stream RobotCommand → CommandResponse`): for each
    `RobotCommand` read from the stream, apply it immediately — under a mutex — to the `Robot`
    (`setWheelVelocities`, `kick`, `dribble`). Return a `CommandResponse{success=true}` when the
    client closes the stream (or `false` with a message on error, e.g. unknown `robot_id` once
    multi-robot exists — for now there's only one robot, so just ignore `robot_id` or reject
    non-zero values with a clear message, your call, document whichever you pick).
  - `SensorStream` (server-streaming: `SensorRequest → stream SensorData`): loop sending the
    latest available camera frame + robot pose/velocity at the `/camera/stream_fps` rate (or
    whenever a new frame becomes available, via a condition variable signaled from the sim
    thread — either approach is fine, prefer whichever is simpler given how Phase 3 ended up
    structured) until the client disconnects.
  - You need a small shared-state object (mutex-guarded) bridging the sim thread (`App::update`,
    running `Robot`/`Camera`) and the gRPC threads (one per active RPC, from gRPC's own thread
    pool): latest camera frame bytes + width/height + timestamp, latest robot pose/velocity,
    and the pending command to apply. Keep it minimal — a plain struct with a `std::mutex` is
    enough, no need for a message queue or anything fancier at this scale.
  - `GrpcServer::stop()` shuts the `grpc::Server` down and joins the background thread.

**Acceptance:** the simulator logs that it's listening on the configured port; a manual test
with `grpcurl` (or the Python client from Phase 6) against `SensorStream` returns frames.

### Phase 6 — Python client tooling

- New top-level `python/` directory:
  - `python/requirements.txt`: `grpcio`, `grpcio-tools`, `opencv-python`, `numpy`.
  - `python/generate_proto.sh`: regenerates the Python gRPC stubs from
    `simulator/proto/simulator.proto` into `python/generated/` (create the dir, add a
    `.gitignore` entry for `python/generated/` — generated code, not source, don't commit it,
    same reasoning as `simulator/build/`).
  - `python/viewer.py`: connects to `localhost:<grpc_port>`, calls `SensorStream` for
    `robot_id=0`, decodes each `SensorData.image_data` into a `(height, width, 3)` `uint8`
    numpy array, and `cv2.imshow`s it in a loop (`cv2.waitKey(1)`, quit on `q` or window close).
    This is the "see it in OpenCV" deliverable — keep it deliberately minimal, it's a demo/test
    client, not a library.
- Document how to run it in `README.md` (new "Python client" section): create a venv, `pip
  install -r python/requirements.txt`, run `generate_proto.sh` once, run `viewer.py` while the
  simulator is running.

**Acceptance:** with the simulator running, `python/viewer.py` opens a window showing the live,
mirror-distorted camera feed, updating as the robot moves in the simulator's own window.

### Phase 7 — wrap up

- Update `ROADMAP.md`: mark items 1 and 2 as done, with a one-line note pointing at this file
  for how they were implemented; leave the rest of the backlog untouched.
- Update `AGENTS.md` only if you introduced a genuinely new convention worth keeping consistent
  (e.g. "shader files live in `simulator/shaders/` and are loaded from disk" if you went that
  route) — don't restate things already covered there.
- Double-check nothing new leaks a machine-specific path, and `python/generated/` /
  `simulator/build/` are both still properly git-ignored.

## Notes for whoever reviews this

Config units stay millimetres in JSON, converted to metres once at load time, per
`AGENTS.md` — the `MirrorProfile` loader is no exception (`hyperbola_a`/`hyperbola_b` etc. are
mm in the JSON, convert on load like everything else in `robot.cpp`/`camera.cpp` already does).
