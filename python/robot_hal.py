#!/usr/bin/env python3
"""Hardware-abstraction layer for the robot: a single coherent interface over the
simulator's gRPC sensor/command API.

`SimRobotHAL` wraps the gRPC `SensorStream` (camera frame, pose, odometry,
lidar, IMU, kicker/capacitor telemetry) and `SendCommand` (drive / dribble /
kicker+capacitor switches) calls behind hardware-agnostic method names, so a
later `HardwareRobotHAL` (real Raspberry Pi backend) can drop in with the same
interface once real hardware exists. This is the seed of the abstraction
described in AGENTS.md, not the robot-control logic itself.

Usage (adding a robot at runtime, then working with it -- see SimulatorClient
for details, e.g. running the simulator with --empty):
    sim = SimulatorClient()
    robot_id = sim.add_robot("configs/robot.json")
    sim.close()

    hal = SimRobotHAL(robot_id=robot_id)
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
import abc
import atexit
import os
import queue
import signal
import socket
import subprocess
import sys
import _thread
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


class SimulatorClient:
    """Thin client for the one simulator-wide operation that isn't scoped to
    an existing robot: adding a new one. Everything else (driving, sensors,
    kicker, IMU, ...) goes through SimRobotHAL once you have a robot_id.

    `config_path` is resolved on the machine RUNNING THE SIMULATOR, relative
    to the simulator process's own working directory -- same convention as
    main.cpp's --robot flag. If the simulator is running on the same machine
    as this script (the normal dev setup), that's just a normal path; if it's
    remote, it must already exist there (this call does not transfer files).

    Typical use, at the start of a control script -- build the whole team up
    front and then just work with the robots:

        sim = SimulatorClient()
        red = sim.add_robot("configs/robot.json")
        blue = sim.add_robot("configs/robot.json", spawn_x=500.0)
        sim.close()

        hal_red = SimRobotHAL(robot_id=red);  hal_red.connect()
        hal_blue = SimRobotHAL(robot_id=blue); hal_blue.connect()
        hal_red.send_velocity(0.5, 0.0, 0.0)

    Robots can be added any time after the simulator process starts (no need
    to launch it with --robot flags first) -- run it with `--empty` if you
    don't want the default single robot it otherwise adds when no --robot
    flags are given. See docs/tasks/multi-robot.md's "Dynamic robot
    creation" section."""

    def __init__(self, host="localhost", port=50051):
        self._channel = grpc.insecure_channel(f"{host}:{port}")
        self._stub = simulator_pb2_grpc.SimulatorStub(self._channel)

    def add_robot(self, config_path, spawn_x=0.0, spawn_z=0.0, spawn_yaw_deg=0.0, timeout=15.0):
        """Adds a new robot to the field from `config_path` and returns its
        robot_id. spawn_x/spawn_z (mm) and spawn_yaw_deg override the config
        file's own /robot/spawn/* (all-zero, the default, means "use the
        file's own spawn point" -- see AddRobotRequest in simulator.proto).
        Blocks until the robot actually exists on the simulator side, so a
        SimRobotHAL(robot_id=...) right after this succeeding will find it.
        Raises RuntimeError if the simulator rejects the request (e.g. a bad
        config_path)."""
        request = simulator_pb2.AddRobotRequest(
            config_path=config_path, spawn_x=spawn_x, spawn_z=spawn_z,
            spawn_yaw_deg=spawn_yaw_deg)
        response = self._stub.AddRobot(request, timeout=timeout)
        if not response.success:
            raise RuntimeError(f"add_robot({config_path!r}) failed: {response.message}")
        return response.robot_id

    def join(self, slot, token=None, timeout=15.0):
        """Claims match slot `slot` (see simulator/configs/match.json); returns
        (robot_id, token, team). Raises RuntimeError if the slot is unknown or
        taken. Pass the token from an earlier join to reconnect to your slot."""
        response = self._stub.Join(
            simulator_pb2.JoinRequest(slot=slot, token=token or ""), timeout=timeout)
        if not response.success:
            raise RuntimeError(f"join({slot!r}) failed: {response.message}")
        return response.robot_id, response.token, response.team

    def list_slots(self):
        """[(slot, team, claimed, robot_id), ...] for the running match."""
        r = self._stub.ListRobots(simulator_pb2.ListRobotsRequest(), timeout=5.0)
        return [(x.slot, x.team, x.claimed, x.robot_id) for x in r.slots]

    def close(self):
        """Releases the channel. Only needed if you're not about to open
        SimRobotHAL connections on the same process anyway -- there's no
        persistent background state to leak, unlike SimRobotHAL.close()."""
        try:
            self._channel.close()
        except Exception:
            pass


_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
_SIM_DIR = os.path.join(_REPO_ROOT, "simulator")
DEFAULT_ROBOT_CONFIG = os.path.join(_SIM_DIR, "configs", "robot.json")


def _free_port():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class Simulator:
    """Launches and owns a simulator process, so a robot script needs no
    manual `./run_simulator.sh`. Comment these lines out and the rest of the
    script runs on the real robot (see create_robot).

        with Simulator() as sim:                # opens the window
            robot = sim.add_robot()             # connected SimRobotHAL
            robot.send_velocity(0.5, 0, 0)
        # leaving the block closes the window

    or without `with`: sim = Simulator().start() ... sim.stop().

    The binary is simulator/build/simulator; if missing it's built first with
    simulator/build.sh (build_if_missing=True). Starts empty (no robots) --
    add them with add_robot(). With exit_on_close=True (default) closing the
    window raises KeyboardInterrupt in your script; poll is_running() instead
    if you prefer (pass exit_on_close=False)."""

    def __init__(self, port=None, build_if_missing=True, host="localhost", exit_on_close=True):
        self._attached = False
        self._exit_on_close = exit_on_close
        self._stopping = False
        self._host = host
        self._port = port
        self._build_if_missing = build_if_missing
        self._proc = None
        self._client = None
        self._robots = []

    @property
    def port(self):
        return self._port

    def start(self, timeout=60.0):
        if self._proc is not None:
            return self
        exe = os.path.join(_SIM_DIR, "build", "simulator")
        if not os.path.exists(exe):
            if not self._build_if_missing:
                raise RuntimeError(f"Simulator binary not found: {exe}")
            print("[Simulator] not built yet, building (simulator/build.sh)...")
            if subprocess.call([os.path.join(_SIM_DIR, "build.sh")]) != 0 or not os.path.exists(exe):
                raise RuntimeError("Simulator build failed (see output above)")
        if self._port is None:
            self._port = _free_port()

        self._proc = subprocess.Popen(
            [exe, "--config-dir", os.path.join(_SIM_DIR, "configs"), "--empty",
             "--port", str(self._port)],
            cwd=_REPO_ROOT)
        atexit.register(self.stop)
        self._stopping = False

        self._client = SimulatorClient(self._host, self._port)
        deadline = time.time() + timeout
        while True:
            if self._proc.poll() is not None:
                raise RuntimeError(f"Simulator exited at startup (code {self._proc.returncode})")
            try:
                self._client._stub.Ping(simulator_pb2.PingRequest(), timeout=1.0)
                break
            except grpc.RpcError:
                if time.time() > deadline:
                    self.stop()
                    raise RuntimeError(f"Simulator did not become ready within {timeout}s")
                time.sleep(0.2)
        if self._exit_on_close:  # only after startup, so a failed start raises normally
            threading.Thread(target=self._watch_window, args=(self._proc,), daemon=True).start()
        return self

    @classmethod
    def connect(cls, host="localhost", port=50051, exit_on_close=True, timeout=30.0):
        """Attaches to a simulator that is ALREADY running (e.g. started with
        `./run_simulator.sh --match configs/match.json`) instead of launching
        one. stop() only disconnects; it never closes that simulator."""
        sim = cls(port=port, host=host, exit_on_close=exit_on_close)
        sim._attached = True
        sim._client = SimulatorClient(host, port)
        deadline = time.time() + timeout
        while True:
            try:
                sim._client._stub.Ping(simulator_pb2.PingRequest(), timeout=1.0)
                break
            except grpc.RpcError:
                if time.time() > deadline:
                    raise RuntimeError(
                        f"No simulator on {host}:{port}. Start it first (./run_simulator.sh).")
                time.sleep(0.3)
        if exit_on_close:
            threading.Thread(target=sim._watch_remote, daemon=True).start()
        return sim

    def join(self, slot, token=None):
        """Claims match slot `slot` and returns a connected SimRobotHAL
        playing as that slot (see simulator/configs/match.json)."""
        robot_id, token, _team = self._client.join(slot, token)
        hal = SimRobotHAL(self._host, self._port, robot_id, token=token)
        hal.connect()
        self._robots.append(hal)
        return hal

    def _watch_remote(self):
        """Attached mode: the process isn't ours, so poll it; end the script
        once the simulator stops answering."""
        misses = 0
        while not self._stopping:
            time.sleep(1.0)
            try:
                self._client._stub.Ping(simulator_pb2.PingRequest(), timeout=1.0)
                misses = 0
            except grpc.RpcError:
                misses += 1
                if misses >= 2 and not self._stopping:
                    self._interrupt_main("simulator closed")
                    return

    def is_running(self):
        """False once the window was closed (X / Esc) or stop() was called."""
        if self._attached:
            if self._stopping or self._client is None:
                return False
            try:
                self._client._stub.Ping(simulator_pb2.PingRequest(), timeout=1.0)
                return True
            except grpc.RpcError:
                return False
        return self._proc is not None and self._proc.poll() is None

    def _watch_window(self, proc):
        """If the user closes the window, interrupt the script's main thread
        (KeyboardInterrupt, so `finally:`/`with` cleanup still runs)."""
        proc.wait()
        if self._stopping:
            return
        self._interrupt_main("window closed")

    def _interrupt_main(self, why):
        print(f"[Simulator] {why}, stopping the script.")
        if os.name == "nt":
            _thread.interrupt_main()
        else:
            os.kill(os.getpid(), signal.SIGINT)  # real signal: also wakes time.sleep()

    def add_robot(self, config_path=None, spawn_x=0.0, spawn_z=0.0, spawn_yaw_deg=0.0):
        """Adds a robot and returns a connected SimRobotHAL. config_path
        defaults to simulator/configs/robot.json; relative paths are resolved
        against the current directory here, on the client."""
        if self._client is None:
            self.start()
        path = os.path.abspath(config_path) if config_path else DEFAULT_ROBOT_CONFIG
        robot_id = self._client.add_robot(path, spawn_x, spawn_z, spawn_yaw_deg)
        hal = SimRobotHAL(self._host, self._port, robot_id)
        hal.connect()
        self._robots.append(hal)
        return hal

    def stop(self):
        """Closes robot connections and, if this object launched it, the
        simulator window. Idempotent."""
        if self._attached:
            if self._client is None:
                return
            self._stopping = True
            for hal in self._robots:
                hal.close()
            self._robots = []
            self._client.close()
            self._client = None
            return
        if self._proc is None:
            return
        self._stopping = True
        for hal in self._robots:
            hal.close()
        self._robots = []
        if self._proc.poll() is None:
            try:
                self._client._stub.Shutdown(simulator_pb2.ShutdownRequest(), timeout=2.0)
            except grpc.RpcError:
                pass
        if self._client is not None:
            self._client.close()
        try:
            self._proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            self._proc.terminate()
            try:
                self._proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                self._proc.kill()
        self._proc = None
        atexit.unregister(self.stop)

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.stop()


def create_robot(sim=None, slot=None, **add_robot_kwargs):
    """The one call robot scripts need; picks the mode itself:

      ROBOT_BACKEND=hw                  -> HardwareRobotHAL (real robot)
      ROBOCUP_SIM=host:port + a slot    -> joins a match on a simulator that is
        (ROBOCUP_SLOT or slot=)            already running, playing as that slot
      otherwise                         -> personal mode: launches its own
                                           simulator with only this robot

    `sim` (an existing Simulator) can still be passed to add a robot to it.
    robot.close() also closes a simulator this call launched."""
    if os.environ.get("ROBOT_BACKEND", "sim").lower() == "hw":
        return HardwareRobotHAL()
    if sim is not None:
        return sim.add_robot(**add_robot_kwargs)
    addr = os.environ.get("ROBOCUP_SIM")
    slot = slot or os.environ.get("ROBOCUP_SLOT")
    if addr:
        if not slot:
            raise ValueError("ROBOCUP_SIM is set but no slot: set ROBOCUP_SLOT or pass slot=")
        host, _, port = addr.rpartition(":")
        sim = Simulator.connect(host or "localhost", int(port))
        hal = sim.join(slot)
        hal._owner = sim
        return hal
    sim = Simulator().start()
    hal = sim.add_robot(**add_robot_kwargs)
    hal._owner = sim
    return hal


class RobotHAL(abc.ABC):
    """Backend-independent robot interface: everything a real robot can do or
    measure. Write robot code against this and it runs unchanged on
    SimRobotHAL (simulator) or HardwareRobotHAL (real robot).

    Ground-truth getters that only a simulator can know (get_pose,
    get_ball_position) are deliberately NOT part of this interface -- they
    live on SimRobotHAL only, so code that needs them can't silently be
    swapped onto hardware."""

    @abc.abstractmethod
    def connect(self, timeout=15.0): ...
    @abc.abstractmethod
    def close(self): ...

    @abc.abstractmethod
    def get_camera_frame(self): ...
    @abc.abstractmethod
    def get_odometry(self): ...
    @abc.abstractmethod
    def get_dribbler_rpm(self): ...
    @abc.abstractmethod
    def get_capacitor_charge(self): ...
    @abc.abstractmethod
    def get_capacitor_voltage(self): ...
    @abc.abstractmethod
    def get_bus_voltage(self): ...
    @abc.abstractmethod
    def get_imu_orientation(self): ...
    @abc.abstractmethod
    def get_imu_acceleration(self): ...
    @abc.abstractmethod
    def get_imu_angular_velocity(self): ...
    @abc.abstractmethod
    def get_lidar_scan(self): ...

    @abc.abstractmethod
    def send_velocity(self, vx, vy, omega): ...
    @abc.abstractmethod
    def dribble(self, speed): ...
    @abc.abstractmethod
    def open_capacitor(self): ...
    @abc.abstractmethod
    def close_capacitor(self): ...
    @abc.abstractmethod
    def open_kicker(self): ...
    @abc.abstractmethod
    def close_kicker(self): ...

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *exc):
        self.close()


class HardwareRobotHAL(RobotHAL):
    """Real-robot backend (Raspberry Pi). Not implemented yet: every method
    raises NotImplementedError. Fill these in with the actual drivers (motor
    controller, camera, IMU, ...) -- the signatures and return formats are
    the ones documented on SimRobotHAL."""

    def _todo(self, *a, **k):
        raise NotImplementedError("HardwareRobotHAL: real-robot driver not written yet")

    connect = close = get_camera_frame = get_odometry = get_dribbler_rpm = _todo
    get_capacitor_charge = get_capacitor_voltage = get_bus_voltage = _todo
    get_imu_orientation = get_imu_acceleration = get_imu_angular_velocity = _todo
    get_lidar_scan = send_velocity = dribble = _todo
    open_capacitor = close_capacitor = open_kicker = close_kicker = _todo


class SimRobotHAL(RobotHAL):
    """Client-side hardware abstraction for one simulated robot.

    The simulator can host several robots at once (see App::addRobot /
    docs/tasks/multi-robot.md on the C++ side, or main.cpp's repeatable
    --robot flag) -- pass the robot's id (0, 1, 2, ... in the order they were
    added on the simulator side) as `robot_id` to talk to that specific one.
    Each id gets an independent command/sensor-stream buffer, so multiple
    SimRobotHAL instances against the same simulator (different robot_id,
    same host/port) never interfere with each other."""

    def __init__(self, host="localhost", port=50051, robot_id=0, token=None, _owner=None):
        self._robot_id = robot_id
        self._metadata = (("x-robot-token", token),) if token else None
        self._owner = _owner        # Simulator to stop when this HAL closes (personal mode)
        self._stream_error = None
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
            if self._stream_error is not None:
                raise RuntimeError(f"Simulator rejected the connection: {self._stream_error.details()}")
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
        if self._owner is not None:
            self._owner.stop()

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
            for data in self._stub.SensorStream(request, metadata=self._metadata):
                with self._lock:
                    self._latest = data
        except grpc.RpcError as e:
            if e.code() == grpc.StatusCode.PERMISSION_DENIED:
                self._stream_error = e

    def _command_generator(self):
        while True:
            cmd = self._cmd_queue.get()
            if cmd is None:
                return
            yield cmd

    def _command_loop(self):
        try:
            self._stub.SendCommand(self._command_generator(), metadata=self._metadata)
        except grpc.RpcError:
            pass
