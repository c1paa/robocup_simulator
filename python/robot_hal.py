#!/usr/bin/env python3
"""Hardware-abstraction layer for the robot: a single coherent interface over the
simulator's gRPC sensor/command API.

`SimRobotHAL` wraps the gRPC `SensorStream` (camera frame, pose, odometry,
lidar) and `SendCommand` (drive / kick / dribble) calls behind hardware-agnostic
method names, so a later `HardwareRobotHAL` (real Raspberry Pi backend) can drop
in with the same interface once real hardware exists. This is the seed of the
abstraction described in AGENTS.md, not the robot-control logic itself.

Usage:
    hal = SimRobotHAL()
    hal.connect()
    frame = hal.get_camera_frame()      # np.ndarray (H, W, 3) or None
    pose  = hal.get_pose()              # (x, y, z, yaw) or None
    scan  = hal.get_lidar_scan()        # np.ndarray (N, 3) [angle, dist, intensity]
    hal.send_velocity(0.5, 0.0, 0.0)
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

        # Persistent command state: vx/vy/omega/kick_power/dribble_speed are
        # independent control channels a real client drives concurrently (e.g.
        # dribbling while approaching the ball). Each send_*/kick/dribble call
        # below updates only its own field(s) and re-sends the *whole* current
        # state, so e.g. calling send_velocity() doesn't silently reset an
        # active dribble_speed back to 0 (RobotCommand's other fields would
        # default to 0 if each call built a fresh message instead).
        self._cmd_lock = threading.Lock()
        self._vx = 0.0
        self._vy = 0.0
        self._omega = 0.0
        self._dribble_speed = 0.0

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
        """Kicker capacitor charge level, 0.0 (empty) to 1.0 (full)."""
        data = self._latest_data()
        if data is None:
            return None
        return data.capacitor_charge

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

    def kick(self, power):
        """Request a kick at the given power (0.0-1.0).

        kick_power is a one-shot trigger, not a persistent channel like
        vx/vy/omega/dribble_speed: it rides along on this single command only,
        so it isn't re-sent (and re-fired) by a later send_velocity()/dribble()
        call.
        """
        with self._cmd_lock:
            self._enqueue(simulator_pb2.RobotCommand(
                robot_id=self._robot_id, vx=self._vx, vy=self._vy,
                omega=self._omega, kick_power=power,
                dribble_speed=self._dribble_speed))

    def dribble(self, speed):
        """Set dribbler speed (-1.0 to 1.0)."""
        with self._cmd_lock:
            self._dribble_speed = speed
            self._send_state_locked()

    # ---- internals ----

    def _send_state_locked(self):
        """Enqueue the current persistent vx/vy/omega/dribble_speed state.
        Caller must hold self._cmd_lock."""
        self._enqueue(simulator_pb2.RobotCommand(
            robot_id=self._robot_id, vx=self._vx, vy=self._vy,
            omega=self._omega, dribble_speed=self._dribble_speed))

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
