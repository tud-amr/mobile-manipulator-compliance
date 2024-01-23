from __future__ import annotations

# fmt: off
position = {0: [-159, 159], 1: [-159, 159], 2: [-159, 159], 3: [-154, 154], 4: [-154, 154], 5: [-154, 154]}
velocity = {0: [0, 250], 1: [0, 250], 2: [0, 250], 3: [0, 250], 4: [0, 250], 5: [0, 250]}
current_motor = {0: [0, 11], 1: [0, 11], 2: [0, 11], 3: [0, 11], 4: [0, 11], 5: [0, 11]}
current_torque_ratios = [0.85, 0.25, 0.85, 1.75, 1.75, 1.75]
torque = {n: [0, current_motor[n][1] / current_torque_ratios[n]] for n in range(len(current_torque_ratios))}
temperature_motor = {0: [0, 60], 1: [0, 60], 2: [0, 60], 3: [0, 60], 4: [0, 60], 5: [0, 60]}
temperature_core = {0: [0, 80], 1: [0, 80], 2: [0, 80], 3: [0, 80], 4: [0, 80], 5: [0, 80]}
# fmt: on

ranges = {
    "position": position,
    "velocity": velocity,
    "current_motor": current_motor,
    "torque": torque,
    "temperature_motor": temperature_motor,
    "temperature_core": temperature_core,
}

actuator_ids = {0: 1, 1: 2, 2: 3, 3: 4, 4: 5, 5: 7}


class Position:
    """Class to define a position for the robot."""

    home: "Position"
    zero: "Position"
    retract: "Position"
    pref: "Position"

    def __init__(self, name: str, position: list[float]) -> None:
        self.name = name
        self.position = position


Position.home = Position("home", [0, -16, 75, 0, -60, 0])
Position.zero = Position("zero", [0, 0, 0, 0, 0, 0])
Position.retract = Position("retract", [0, 20, 150, -90, -40, -90])
Position.pref = Position("pref", [0, 20, 90, 0, 0, 0])
# old: [0, -20, 100, -90, -60, 0]
