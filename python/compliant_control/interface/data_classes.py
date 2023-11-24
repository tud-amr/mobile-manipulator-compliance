from dataclasses import dataclass


@dataclass
class Joint:
    """Data of a joint."""

    index: int

    # continuous feedback:
    pos: float = 0
    vel: float = 0
    eff: float = 0

    # state:
    active: bool = False
    mode: str = "?"
    ratio: float = 0
    friction: float = 0

    @property
    def name(self) -> str:
        """Return the name of the joint."""
        return f"joint{self.index}"

    def is_active(self) -> bool:
        """Return wether the joint is active."""
        return self.active

    def get_mode(self) -> str:
        """Return the mode of the joint."""
        return self.mode[:3]

    def get_ratio(self) -> str:
        """Return the ratio of the joint."""
        return str(self.ratio)

    def get_friction(self) -> str:
        """Return the friction of the joint."""
        return str(round(self.friction, 3))


@dataclass
class Wheel:
    """Data of a wheel."""

    index: int

    # continuous feedback:
    encoder_position: float = 0
    zero_position: float = 0
    vel: float = 0
    eff: float = 0

    @property
    def name(self) -> str:
        """Return the name of the wheel."""
        return f"wheel{self.index}"

    @property
    def pos(self) -> float:
        """Return the current position."""
        return self.encoder_position - self.zero_position

    def reset(self) -> None:
        """Reset the position."""
        self.zero_position = self.encoder_position


@dataclass
class Rates:
    """Object to collect update rate."""

    kin: int = 0
    din: int = 0
    con: int = 0

    @property
    def names(self) -> list[str]:
        """Get the names."""
        return ["kin", "din", "con"]

    def value(self, name: str) -> str:
        """Get the value."""
        return str(getattr(self, name))


@dataclass
class State:
    """Data of the robot state."""

    mode: str = "waiting"
    update_rate: int = 0
    servoing: str = "?"
    comp_grav: bool = False
    comp_fric: bool = False
    imp_arm: bool = False
    imp_null: bool = False
    imp_base: bool = False
    automove_target: bool = False

    def HLT(self) -> bool:
        """Returns whether the current mode is HLT."""
        return self.mode == "HLT"

    def HLC(self) -> bool:
        """Returns whether the current mode is HLC."""
        return self.mode == "HLC"

    def LLC(self) -> bool:
        """Returns whether the current mode is LLC."""
        return self.mode == "LLC"

    def LLC_task(self) -> bool:
        """Returns whether a LLC task is active."""
        return self.mode == "LLC_task"

    def get_rate(self) -> str:
        """Get the update rate."""
        return str(self.update_rate)

    def get_servoing(self) -> str:
        """Get the servoing mode."""
        return self.servoing

    def get_comp_grav(self) -> bool:
        """Return whether gravity compensation is enabled."""
        return self.comp_grav

    def get_comp_fric(self) -> bool:
        """Return whether friction compensation is enabled."""
        return self.comp_fric

    def get_imp_arm(self) -> bool:
        """Return whether arm impedance is enabled."""
        return self.imp_arm

    def get_imp_null(self) -> bool:
        """Return whether null space impedance is enabled."""
        return self.imp_null

    def get_imp_base(self) -> bool:
        """Return whether base impedance is enabled."""
        return self.imp_base

    def get_automove_target(self) -> bool:
        """Return whether automove target is enabled."""
        return self.automove_target
