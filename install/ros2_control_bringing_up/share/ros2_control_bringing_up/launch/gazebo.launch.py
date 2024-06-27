import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    package_share_dir = get_package_share_directory("ros2_control_bringing_up")
    urdf_file = os.path.join(package_share_dir, "urdf", "fum6r_new_corsys.urdf")
    
    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=["gazebo","-s","libgazebo_ros_factory.so","-u","--verbose",],
                output="screen",
            ),
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=["-entity","ros2_control_bringing_up","-b","-file", urdf_file,],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                arguments=[urdf_file],
            ),
            

        ]
    )