import os
import time
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import xacro

def generate_launch_description():

    gazebo_ros2_control_path = os.path.join(
        get_package_share_directory('ros2_control_bringup'))

    xacro_file = os.path.join(gazebo_ros2_control_path,
                              'sdf',
                              'ball.sdf')
    
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)

    gz_spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=[ '-string', doc.toxml(),
                                   '-name', 'ball',
                                   '-allow_renaming', 'true'],
                        output='screen')

    return LaunchDescription([
        gz_spawn_entity, 
    ])
