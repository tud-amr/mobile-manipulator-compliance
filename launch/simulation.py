from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> None:
    """Generate the launch description."""
    return LaunchDescription(
        [
            Node(
                package="simulation",
                executable="simulation_node",
            ),
            Node(
                package="user_interface",
                executable="user_interface_node",
            ),
            Node(
                package="kinova_driver",
                executable="kinova_driver_node",
            ),
            Node(
                package="dingo_driver_simulation",
                executable="dingo_driver_simulation_node",
            ),
            Node(
                package="controller",
                executable="controller_node",
            ),
        ]
    )
