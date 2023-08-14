import subprocess
from user_interface.launcher import Launcher
from user_interface.mujoco_viewer import MujocoViewer
from kinova import utilities
from kinova.kortex_client import KortexClient
from kinova.kortex_client_mock import KortexClientMock
from kinova.kortex_client_simulation import KortexClientSimulation
from compliant_controller.state import State


def ip_available() -> bool:
    """Check if robot is available."""
    return (
        subprocess.call(
            "ping -c 1 -W 0.1 " + utilities.DEFAULT_IP,
            shell=True,
            stdout=subprocess.DEVNULL,
        )
        == 0
    )


def start_interface(client: KortexClient, mujoco_viewer: MujocoViewer) -> None:
    """Start the graphical user interface."""
    Launcher(client, mujoco_viewer)
    client.stop_refresh_loop()


def main() -> int:
    """Connect with the robot or the simulation and start the graphical user interface."""
    simulate = True
    mujoco_viewer = MujocoViewer()
    state = State(mujoco_viewer)
    if not ip_available():
        if simulate:
            client = KortexClientSimulation(state, mujoco_viewer)
        else:
            client = KortexClientMock(state, 6)
        start_interface(client, mujoco_viewer)
        return 1

    with utilities.DeviceConnection.createTcpConnection() as router, utilities.DeviceConnection.createUdpConnection() as real_time_router:
        # Create required services
        client = KortexClient(state, router=router, real_time_router=real_time_router)
        start_interface(client, mujoco_viewer)
        return 1


if __name__ == "__main__":
    exit(main())
