from numpy import ndarray

class DriverManager:
    """Manages the canbus connections."""

    def __init__(self, canbus_name: str) -> None:
        """Define the canbus name."""
    @property
    def position(self) -> ndarray:
        """Get the vector of all wheel positions."""
    @property
    def torque(self) -> ndarray:
        """Get the vector of all wheel torques."""
    def connect_gateway(self) -> None:
        """Connect the gateway."""
    def start_canread_loop(self) -> None:
        """Start the canread loop in a new thread."""
    def start_update_loop(self) -> None:
        """Start the update loop in a new thread."""
    def set_command(self, command: ndarray) -> None:
        """Set the command."""
