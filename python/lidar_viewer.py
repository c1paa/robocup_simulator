#!/usr/bin/env python3
"""Live 2D lidar viewer: reads the robot's simulated LD06 scan over gRPC (via
robot_hal.SimRobotHAL) and plots it as a real-time polar scatter, robot at
the center, angle=0 (body-forward) pointing up. Run the simulator first,
then:

    python lidar_viewer.py [host] [port]
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from robot_hal import SimRobotHAL


def main():
    host = sys.argv[1] if len(sys.argv) > 1 else "localhost"
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 50051

    hal = SimRobotHAL(host=host, port=port)
    hal.connect()

    fig = plt.figure("LiDAR", figsize=(6, 6))
    ax = fig.add_subplot(111, projection="polar")
    ax.set_theta_zero_location("N")   # body-forward (angle=0) points up
    ax.set_theta_direction(-1)        # positive angle sweeps clockwise on screen
    ax.set_rlabel_position(135)
    ax.set_title("Robot-frame LiDAR scan (forward = up)")
    scatter = ax.scatter([], [], s=4, c=[], cmap="viridis", vmin=0.0, vmax=1.0)

    def update(_frame):
        scan = hal.get_lidar_scan()
        if scan is None or len(scan) == 0:
            return (scatter,)
        angles = scan[:, 0]
        distances = scan[:, 1]
        intensities = scan[:, 2]
        scatter.set_offsets(np.column_stack((angles, distances)))
        scatter.set_array(intensities)
        r_max = max(1.0, float(distances.max()) * 1.1)
        ax.set_rmax(r_max)
        return (scatter,)

    ani = animation.FuncAnimation(fig, update, interval=50, cache_frame_data=False)
    try:
        plt.show()
    finally:
        hal.close()


if __name__ == "__main__":
    main()
