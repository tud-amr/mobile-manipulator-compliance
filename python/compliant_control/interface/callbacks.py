class Callbacks:
    """Contains the callbacks."""

    def __init__(self) -> None:
        self.buttons = self.empty
        self.joystick = self.empty

    def empty(self, *args: any) -> None:
        """Empty method."""
