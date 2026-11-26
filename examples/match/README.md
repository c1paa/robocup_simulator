# Match example

Terminal 1 -- the empty simulator (slots from `simulator/configs/match.json`), phase "waiting":

    ./run_simulator.sh

Terminals 2..5 -- the same script, different ID (0 yellow_1, 1 yellow_2, 2 blue_1, 3 blue_2):

    python examples/main.py ID=0
    python examples/main.py ID=2

Press **P** in the simulator window to start / pause / resume. Closing the window
stops every program; a program that ends removes its robot from the field.
Without `ID=` the same script opens its own personal simulator.
