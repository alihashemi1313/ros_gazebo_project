from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    position_publisher_node = Node(
        package="ros2_control_bringing_up",
        executable="position_target",
        name="position_publisher",
    )

    position_subscriber_node = Node(
        package="motion_planning",
        executable="position_subscriber",
        name="position_subscriber",
    )

    ld.add_action(position_publisher_node)
    ld.add_action(position_subscriber_node)

    return ld
