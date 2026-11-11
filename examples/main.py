"""Robot script: the same file runs in the simulator (alone or in a match)
or on the real robot.

  python examples/main.py                      -> opens its own simulator, just this robot
  ROBOCUP_SIM=localhost:50051 ROBOCUP_SLOT=red_1 python examples/main.py
                                               -> plays as red_1 in a match already
                                                  running (./run_simulator.sh --match configs/match.json)
  ROBOT_BACKEND=hw python examples/main.py     -> real robot
"""
import time

from robot_hal import create_robot

robot = create_robot()
try:
    robot.send_velocity(0.3, 0.0, 0.0)
    time.sleep(2.0)
    print("odometry:", robot.get_odometry())
    robot.send_velocity(0.0, 0.0, 0.0)
except KeyboardInterrupt:      # simulator window was closed
    pass
finally:
    robot.close()              # personal mode: also closes the simulator window
