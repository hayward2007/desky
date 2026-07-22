"""Demo motion sequence for the desky arm — single source of truth shared by
main.py (drives the real actuators) and simulate.py (3D preview, no hardware).

Each step's `positions` maps DYNAMIXEL id -> servo degree (0-300) to command;
ids not mentioned in a step keep whatever they were last commanded to. `hold`
is how long (seconds) main.py waits after issuing that step's goto() calls
before moving to the next step.
"""

DEMO_SEQUENCE = [
    {"positions": {1: 180, 2: 180, 3: 180, 4: 180, 5: 180}, "hold": 2.0},
    {"positions": {3: 170, 4: 220, 5: 280}, "hold": 5.0},
]
