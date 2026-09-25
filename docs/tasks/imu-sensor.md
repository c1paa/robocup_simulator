# IMU sensor (BNO055-modeled)

Adds a simulated Bosch BNO055 9-axis absolute orientation sensor, matching how the real part's own
onboard sensor fusion behaves rather than a naive "integrate the gyro" model. New class
`ImuSensor` (`simulator/src/imu_sensor.h`/`.cpp`), peer to `LidarSensor` — owned by `App`, updated
every frame but internally gated to its own output rate, same pattern as `LidarSensor`'s
`scan_frequency` being independent of the render/physics tick.

## Why fused orientation, not gyro integration

The real BNO055 is a "System in Package": a triaxial accelerometer, triaxial gyroscope, triaxial
magnetometer, and an onboard ARM Cortex-M0+ running Bosch's own sensor-fusion firmware, all in one
part. In its NDOF fusion mode it outputs already-fused Euler angles (`VECTOR_EULER`), a gravity-
removed linear acceleration (`VECTOR_LINEARACCEL`), and raw angular velocity (`VECTOR_GYROSCOPE`) —
not just a raw gyro a host MCU would have to integrate itself. Critically, because yaw is anchored to
the magnetometer rather than purely integrated from the gyro, **heading does not accumulate drift**
the way a plain gyro-integration approach would — only small, bounded per-sample noise, exactly what
the user asked for ("считаются достаточно точно и помех почти нет... накопления ошибки например по
компасу не будет").

This is modeled directly rather than simulated via gyro integration + a magnetometer-correction
step: `ImuSensor::sample()` reads the robot's *true* yaw straight from `Robot::orientation()` and
adds Gaussian noise, instead of integrating `angularVelocity()` over time (which would have to
reimplement drift-correction logic just to then simulate away the drift it introduced — pure
unnecessary complexity for the same net result). Roll and pitch are always exactly 0 before noise,
for a real physical reason specific to this robot: `Robot`'s rigid body has
`setAngularFactor(btVector3(0,1,0))` — yaw-only, no tipping (see `robot.cpp`) — so a sensor mounted
flat on this chassis genuinely never has real roll/pitch to report, matching what the real part
would read on a robot that never leaves the ground.

## Reference specs (Bosch BNO055 datasheet)

Used as the basis for default noise figures, not reproduced with datasheet-level precision (see
"Known simplifications" below):

- Fusion output data rate: up to 100 Hz (`/robot/imu/update_rate_hz`, default 100).
- Heading (yaw) accuracy: ~2.5° typical (`/robot/imu/yaw_noise_std_deg`, default 2.5).
- Roll/pitch accuracy: ~1° typical for the fused output (`/robot/imu/roll_pitch_noise_std_deg`,
  default 1.0).
- Accelerometer output noise density: 150-190 µg/√Hz typical/max — used as the physical basis for
  `/robot/imu/accel_noise_std` (default 0.03 m/s², a flat per-sample std rather than a noise-
  density-times-bandwidth derivation, since the exact fusion-stage filtering bandwidth isn't
  published).
- Gyroscope range: ±2000°/s (16-bit) — not directly used (the model reports true angular velocity
  plus noise, not a quantized/ranged raw value); `/robot/imu/gyro_noise_std_deg` (default 0.3°/s)
  is a physically-reasonable noise figure for the fused output, not read off the datasheet's raw
  gyro noise-density table directly.

## Model (`ImuSensor::sample`, called once per `1/update_rate_hz`)

- **Orientation**: `roll = noise`, `pitch = noise`, `yaw = robot.orientation() + noise` — see
  "Why fused orientation" above.
- **Angular velocity**: `gyro = (0, robot.angularVelocity(), 0) + noise` on all three axes — the
  yaw-rate component is exact (read directly from Bullet, not integrated), matching
  `SensorData.angular_velocity` plus sensor noise; roll/pitch rates are always exactly 0 before
  noise for the same yaw-only-body reason as above.
- **Linear acceleration**: reports the real part's `VECTOR_LINEARACCEL` semantics (gravity already
  removed), not raw `VECTOR_ACCELEROMETER` (which would read +g on the up axis at rest). Computed
  as true coordinate acceleration, finite-differenced from `Robot::linearVelocityWorld()` between
  samples (Bullet doesn't expose acceleration directly, and this is the only practical way to get
  it without duplicating `Robot`'s own force accounting inside `ImuSensor`), then rotated world →
  body frame using the exact same transform `Robot::applyDriveForces` uses for `linX`/`linZ`
  (`AGENTS.md`'s yaw convention — copied, not re-derived by hand). The vertical (body `+Y`)
  component is always 0 before noise: this sim has no jump/tip model, so true world-`Y` coordinate
  acceleration is always exactly 0, which is itself the physically correct `VECTOR_LINEARACCEL`
  reading for a sensor that never leaves a level surface.

## Config (`/robot/imu` in `robot.json`)

```jsonc
"imu": {
    "update_rate_hz": 100,
    "roll_pitch_noise_std_deg": 1.0,
    "yaw_noise_std_deg": 2.5,
    "accel_noise_std": 0.03,
    "gyro_noise_std_deg": 0.3
}
```

## Proto / API

`SensorData` gained nine fields: `imu_roll`/`imu_pitch`/`imu_yaw` (radians), `imu_accel_x/y/z`
(m/s²), `imu_gyro_x/y/z` (rad/s) — see `simulator/proto/simulator.proto`. `SimRobotHAL` exposes
three getters mirroring the real driver's own `getVector(VECTOR_*)` call shape rather than one
combined struct, since that's how real BNO055 client code is actually written (three separate
calls, one per vector):

```python
roll, pitch, yaw = hal.get_imu_orientation()
ax, ay, az = hal.get_imu_acceleration()
gx, gy, gz = hal.get_imu_angular_velocity()
```

## Known simplifications, stated honestly

- Noise is i.i.d. Gaussian per sample, not a proper bias-instability/random-walk noise model (real
  MEMS gyros have a slowly-wandering bias term on top of white noise) — judged not worth the extra
  state for a sensor whose main property that matters here (bounded, driftless yaw) already holds.
- No temperature dependence, no calibration-state modeling (the real BNO055 needs a calibration
  routine on power-up; this always reports as if fully calibrated).
- No magnetic-disturbance modeling (real magnetometer-based heading can be thrown off by nearby
  ferrous material/motors) — the simulated heading noise is a flat, constant-magnitude figure.
- `accel_noise_std`/`gyro_noise_std_deg` are reasonable figures derived from the datasheet's stated
  noise densities, not an exact reproduction of the datasheet's own filtering/bandwidth math — see
  "Reference specs" above.
