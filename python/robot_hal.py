#!/usr/bin/env python3
"""Hardware-abstraction layer for the robot: a single coherent interface over the
simulator's gRPC sensor/command API.

`SimRobotHAL` wraps the gRPC `SensorStream` (camera frame, pose, odometry,
lidar, IMU, kicker/capacitor telemetry) and `SendCommand` (drive / dribble /
kicker+capacitor switches) calls behind hardware-agnostic method names, so a
later `HardwareRobotHAL` (real Raspberry Pi backend) can drop in with the same
interface once real hardware exists. This is the seed of the abstraction
described in AGENTS.md, not the robot-control logic itself.

Usage:
    hal = SimRobotHAL()
    hal.connect()
    frame = hal.get_camera_frame()      # np.ndarray (H, W, 3) or None
    pose  = hal.get_pose()              # (x, y, z, yaw) or None
    scan  = hal.get_lidar_scan()        # np.ndarray (N, 3) [angle, dist, intensity]
    hal.send_velocity(0.5, 0.0, 0.0)

    # Kicker: charge, then fire, each with your own timing (see
    # open_capacitor()/open_kicker() docstrings -- never open both at once).
    hal.open_capacitor()
    time.sleep(0.5)
    hal.close_capacitor()
    hal.open_kicker()
    time.sleep(0.02)
    hal.close_kicker()

    hal.close()
"""
import os
import sys
import queue
import threading
import time


def _ensure_generated_stubs():
    """Generate generated/simulator_pb2*.py from simulator.proto if it's
    missing, or stale (the .proto changed since the stub was last built) --
    so a fresh `git clone` + `pip install -e python/` needs no separate
    `generate_proto.sh` step; import alone is enough. Only regenerates when
    actually needed (checked by mtime), same output generate_proto.sh
    produces, so running that script explicitly still works too.

    Requires the source .proto file to exist at its normal repo-relative
    location (../simulator/proto/simulator.proto) -- true for both a plain
    clone and an editable pip install (`pip install -e`), since neither
    copies this file out of the repo tree. If the .proto isn't there (e.g. a
    non-editable install shipped without the rest of the repo) but a stub
    already exists, the existing stub is trusted as-is instead of erroring.
    """
    here = os.path.dirname(os.path.abspath(__file__))
    gen_dir = os.path.join(here, "generated")
    stub = os.path.join(gen_dir, "simulator_pb2.py")
    proto = os.path.abspath(os.path.join(here, "..", "simulator", "proto", "simulator.proto"))

    stub_exists = os.path.exists(stub)
    proto_exists = os.path.exists(proto)
    if stub_exists and (not proto_exists or os.path.getmtime(stub) >= os.path.getmtime(proto)):
        return
    if not proto_exists:
        if stub_exists:
            return
        raise RuntimeError(
            f"No generated gRPC stubs at {stub}, and the .proto source ({proto}) "
            "doesn't exist either. robot_hal.py needs to run from inside the "
            "robocup_simulator repo (a plain clone or `pip install -e`), not a "
            "standalone copy of just this file."
        )

    os.makedirs(gen_dir, exist_ok=True)
    from grpc_tools import protoc
    proto_dir = os.path.dirname(proto)
    args = [
        "protoc",
        f"--proto_path={proto_dir}",
        f"--python_out={gen_dir}",
        f"--grpc_python_out={gen_dir}",
        proto,
    ]
    if protoc.main(args) != 0:
        raise RuntimeError("Failed to generate gRPC stubs from simulator.proto")


_ensure_generated_stubs()
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "generated"))

import grpc
import numpy as np

import simulator_pb2
import simulator_pb2_grpc


class SimRobotHAL:
    """Client-side hardware abstraction for the simulated robot (robot_id 0)."""

    def __init__(self, host="localhost", port=50051, robot_id=0):
        self._robot_id = robot_id
        self._channel = grpc.insecure_channel(f"{host}:{port}")
        self._stub = simulator_pb2_grpc.SimulatorStub(self._channel)

        self._lock = threading.Lock()
        self._latest = None  # most recent SensorData

        self._cmd_queue = queue.Queue()
        self._closed = False
        self._sensor_thread = None
        self._command_thread = None

        # Persistent command state: vx/vy/omega/dribble_speed/capacitor_open/
        # kicker_open are independent control channels a real client drives
        # concurrently (e.g. dribbling while approaching the ball, or charging
        # the capacitor while driving into position). Each send_*/dribble/
        # open_*/close_* call below updates only its own field(s) and
        # re-sends the *whole* current state, so e.g. calling send_velocity()
        # doesn't silently reset an active dribble_speed or open capacitor
        # switch back to 0/False (RobotCommand's other fields would default
        # to 0/False if each call built a fresh message instead).
        self._cmd_lock = threading.Lock()
        self._vx = 0.0
        self._vy = 0.0
        self._omega = 0.0
        self._dribble_speed = 0.0
        self._capacitor_open = False
        self._kicker_open = False

    # ---- lifecycle ----

    def connect(self, timeout=15.0):
        """Start the background sensor reader and command stream, and block
        until the simulator is actually ready: the gRPC channel is up AND the
        first SensorData message has arrived. Prints progress to the console
        while waiting (connecting can take a few seconds while the simulator
        window is still starting up). Raises RuntimeError if `timeout` seconds
        pass without either step completing.
        """
        if self._sensor_thread is not None:
            return

        print(f"[SimRobotHAL] connecting to simulator...")
        try:
            grpc.channel_ready_future(self._channel).result(timeout=timeout)
        except grpc.FutureTimeoutError:
            raise RuntimeError(
                f"Could not reach the simulator within {timeout}s. Is it running? "
                "(./run_simulator.sh)")
        print("[SimRobotHAL] channel connected, waiting for first sensor frame...")

        self._sensor_thread = threading.Thread(target=self._sensor_loop, daemon=True)
        self._sensor_thread.start()
        self._command_thread = threading.Thread(target=self._command_loop, daemon=True)
        self._command_thread.start()

        deadline = time.time() + timeout
        while self._latest_data() is None:
            if time.time() > deadline:
                raise RuntimeError(
                    f"Connected to the simulator but received no sensor data within "
                    f"{timeout}s (robot_id={self._robot_id} mismatch?).")
            time.sleep(0.05)
        print("[SimRobotHAL] connected, first sensor frame received.")

    def close(self):
        """Stop the background streams and release the channel."""
        if self._closed:
            return
        self._closed = True
        self._cmd_queue.put(None)
        try:
            self._channel.close()
        except Exception:
            pass

    # ---- sensor accessors ----

    def _latest_data(self):
        with self._lock:
            return self._latest

    def get_camera_frame(self):
        """Latest camera image as an (H, W, 3) uint8 ndarray, or None."""
        data = self._latest_data()
        if data is None or not data.image_data:
            return None
        expected = data.image_width * data.image_height * 3
        if len(data.image_data) != expected:
            return None
        return np.frombuffer(data.image_data, dtype=np.uint8).reshape(
            (data.image_height, data.image_width, 3))

    def get_pose(self):
        """Ground-truth pose (x, y, z, yaw), or None."""
        data = self._latest_data()
        if data is None:
            return None
        return (data.pos_x, data.pos_y, data.pos_z, data.yaw)

    def get_odometry(self):
        """Dead-reckoning odometry estimate (x, z, yaw), or None."""
        data = self._latest_data()
        if data is None:
            return None
        return (data.odom_x, data.odom_z, data.odom_yaw)

    def get_ball_position(self):
        """Ball ground-truth position (x, y, z), or None."""
        data = self._latest_data()
        if data is None:
            return None
        return (data.ball_pos_x, data.ball_pos_y, data.ball_pos_z)

    def get_dribbler_rpm(self):
        """Dribbler motor's actual speed in RPM (signed like dribble_speed)."""
        data = self._latest_data()
        if data is None:
            return None
        return data.dribbler_rpm

    def get_capacitor_charge(self):
        """Kicker capacitor charge level, 0.0 (empty) to 1.0 (full) --
        capacitor_voltage / charge_voltage (see get_capacitor_voltage())."""
        data = self._latest_data()
        if data is None:
            return None
        return data.capacitor_charge

    def get_capacitor_voltage(self):
        """Kicker capacitor voltage in volts (0.0 up to /robot/capacitor/charge_voltage)."""
        data = self._latest_data()
        if data is None:
            return None
        return data.capacitor_voltage

    def get_bus_voltage(self):
        """Robot's current (sagged) power bus voltage in volts. Drops while
        the capacitor is charging, and drops severely if open_capacitor()
        and open_kicker() are both active at once (a real short circuit --
        see open_kicker())."""
        data = self._latest_data()
        if data is None:
            return None
        return data.bus_voltage

    def get_imu_orientation(self):
        """Fused absolute orientation (roll, pitch, yaw) in radians, BNO055-
        modeled: not integrated from the gyro, so yaw does not accumulate
        drift the way raw gyro integration would (magnetometer-anchored
        heading) -- only small, bounded per-sample noise. Mirrors the real
        driver's getVector(VECTOR_EULER)."""
        data = self._latest_data()
        if data is None:
            return None
        return (data.imu_roll, data.imu_pitch, data.imu_yaw)

    def get_imu_acceleration(self):
        """Linear acceleration (x, y, z) in m/s^2, robot body frame, gravity
        already removed -- mirrors the real driver's
        getVector(VECTOR_LINEARACCEL)."""
        data = self._latest_data()
        if data is None:
            return None
        return (data.imu_accel_x, data.imu_accel_y, data.imu_accel_z)

    def get_imu_angular_velocity(self):
        """Angular velocity (x, y, z) in rad/s, robot body frame -- mirrors
        the real driver's getVector(VECTOR_GYROSCOPE)."""
        data = self._latest_data()
        if data is None:
            return None
        return (data.imu_gyro_x, data.imu_gyro_y, data.imu_gyro_z)

    def get_lidar_scan(self):
        """Latest completed lidar scan as an (N, 3) ndarray [angle, distance,
        intensity] (angle in radians, robot body frame), or None."""
        data = self._latest_data()
        if data is None or not data.lidar_points:
            return None
        return np.array(
            [(p.angle, p.distance, p.intensity) for p in data.lidar_points],
            dtype=np.float32)

    # ---- command senders ----

    def send_velocity(self, vx, vy, omega):
        """Send a body-frame drive command (vx forward, vy lateral, omega yaw)."""
        with self._cmd_lock:
            self._vx, self._vy, self._omega = vx, vy, omega
            self._send_state_locked()

    def dribble(self, speed):
        """Set dribbler speed (-1.0 to 1.0)."""
        with self._cmd_lock:
            self._dribble_speed = speed
            self._send_state_locked()

    def open_capacitor(self):
        """Close the charging switch: the boost converter starts pushing
        current into the kicker capacitor through the charge-limiting
        resistor. You control how long this stays open -- call
        close_capacitor() yourself when you've charged enough (poll
        get_capacitor_voltage()/get_capacitor_charge() to know when).

        Opening this at the same time as open_kicker() is a real short
        circuit (see open_kicker()) -- don't do both at once."""
        with self._cmd_lock:
            self._capacitor_open = True
            self._send_state_locked()

    def close_capacitor(self):
        """Open the charging switch. The capacitor holds whatever charge it
        has (slow leakage only)."""
        with self._cmd_lock:
            self._capacitor_open = False
            self._send_state_locked()

    def open_kicker(self):
        """Close the discharge switch: the capacitor dumps into the solenoid
        coil, driving the armature toward the ball. You control the fire
        duration yourself -- call close_kicker() when done (a real strike
        completes in a few milliseconds; holding this open longer than that
        does nothing further once the capacitor is spent).

        Opening this while open_capacitor() is also open is a real short
        circuit: the discharge path bypasses the charge resistor's current
        limiting, so the bus voltage collapses hard (see get_bus_voltage())
        and the capacitor doesn't charge or fire coherently. This is exactly
        what happens on real hardware if firmware mismanages the two gates --
        the simulator doesn't prevent it, it simulates the consequence."""
        with self._cmd_lock:
            self._kicker_open = True
            self._send_state_locked()

    def close_kicker(self):
        """Open the discharge switch."""
        with self._cmd_lock:
            self._kicker_open = False
            self._send_state_locked()

    # ---- internals ----

    def _send_state_locked(self):
        """Enqueue the current persistent vx/vy/omega/dribble_speed/
        capacitor_open/kicker_open state. Caller must hold self._cmd_lock."""
        self._enqueue(simulator_pb2.RobotCommand(
            robot_id=self._robot_id, vx=self._vx, vy=self._vy,
            omega=self._omega, dribble_speed=self._dribble_speed,
            capacitor_charge_open=self._capacitor_open,
            kicker_open=self._kicker_open))

    def _enqueue(self, cmd):
        if self._closed:
            return
        self._cmd_queue.put(cmd)

    def _sensor_loop(self):
        request = simulator_pb2.SensorRequest(robot_id=self._robot_id)
        try:
            for data in self._stub.SensorStream(request):
                with self._lock:
                    self._latest = data
        except grpc.RpcError:
            pass

    def _command_generator(self):
        while True:
            cmd = self._cmd_queue.get()
            if cmd is None:
                return
            yield cmd

    def _command_loop(self):
        try:
            self._stub.SendCommand(self._command_generator())
        except grpc.RpcError:
            pass
