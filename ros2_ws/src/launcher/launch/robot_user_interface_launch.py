from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> None:
    """Generate the launch description."""
    return LaunchDescription(
        [
            Node(
                package="user_interface",
                executable="user_interface_node",
                parameters=[{"visualize": True}],
            ),
        ]
    )
