"""Robot script that runs in the simulator or on the real robot.

Simulator: just run it.   Real robot: ROBOT_BACKEND=hw python examples/main.py
(or delete the three simulator lines below).
"""
import time

from robot_hal import Simulator, create_robot

sim = Simulator().start()          # <-- comment out for the real robot
robot = create_robot(sim)          # real robot: robot = HardwareRobotHAL()
robot.connect()

try:
    robot.send_velocity(0.3, 0.0, 0.0)
    time.sleep(2.0)
    print("odometry:", robot.get_odometry())
    robot.send_velocity(0.0, 0.0, 0.0)
finally:
    robot.close()
    sim.stop()                     # <-- comment out for the real robot
