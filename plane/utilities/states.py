from enum import Enum


class States(Enum):
    STOP = -2
    IDLE = 0
    COLLECT = 1  # This state can only be entered by a command from the ground
    GPS_AVAIL = 2  # The gps can be loaded via the algorithm, or a manual input
    ARMING_PADA = 3  # Arming starts every time the gps is updated
    READY = 4
    DROPPED = 5


class StateMachine:
    def __init__(self, initial_state):
        self.state = initial_state
        self.last_state = initial_state
        self.states_lookup = {
            "STOP": States.STOP,
            "IDLE": States.IDLE,
            "COLLECT": States.COLLECT,
        }

    def __str__(self):
        return self.name

    def __repr__(self):
        return self.name
