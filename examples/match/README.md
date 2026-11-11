# Match example

Terminal 1 -- the simulator, empty, with the slots from `simulator/configs/match.json`:

    ./run_simulator.sh --match simulator/configs/match.json

Terminals 2..N -- one robot program each (any scripts; here the same one):

    ROBOCUP_SIM=localhost:50051 ROBOCUP_SLOT=red_1  python examples/main.py
    ROBOCUP_SIM=localhost:50051 ROBOCUP_SLOT=blue_1 python examples/main.py

Each program only controls its own slot's robot (token-protected). The same
script without those variables opens its own personal simulator instead.
