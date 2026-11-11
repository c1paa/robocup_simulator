"""Robot script: the same file runs in the simulator (alone or in a match)
or on the real robot.

  python examples/main.py                      -> opens its own simulator, just this robot
  ROBOCUP_SIM=localhost:50051 ROBOCUP_SLOT=red_1 python examples/main.py
                                               -> plays as red_1 in a match already
                                                  running (./run_simulator.sh --match simulator/configs/match.json;
                                                  press Enter in its window to start)
  ROBOT_BACKEND=hw python examples/main.py     -> real robot
"""
import time

from robot_hal import create_robot

robot = create_robot()
try:
    robot.wait_for_start()              # match: returns when the game starts
    while robot.is_running():           # False when the simulator is closing
        robot.send_velocity(0.3, 0.0, 0.0)
        time.sleep(0.05)
finally:
    robot.send_velocity(0.0, 0.0, 0.0)
    robot.close()                       # personal mode: also closes the simulator window
