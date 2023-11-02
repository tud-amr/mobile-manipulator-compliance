from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> None:
    """Generate the launch description."""
    return LaunchDescription(
        [
            Node(
                package="kinova_driver",
                executable="kinova_driver_node",
            ),
            Node(
                package="dingo_driver",
                executable="dingo_driver_node",
            ),
            Node(
                package="controller",
                executable="controller_node",
            ),
        ]
    )
