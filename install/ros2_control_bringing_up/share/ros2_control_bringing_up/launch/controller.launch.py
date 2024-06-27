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
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
             )

    gazebo_ros2_control_path = os.path.join(
        get_package_share_directory('ros2_control_bringing_up'))

    xacro_file = os.path.join(gazebo_ros2_control_path,
                              'urdf',
                              'fum6r.urdf')
    
    world_file = os.path.join(gazebo_ros2_control_path,
                              'world',
                              'FUM6R_u.world')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[params,{"use_sim_time": True}]
    )

    gz_spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=[ '-string', doc.toxml(),
                                   '-name', 'fum6r',
                                   '-allow_renaming', 'true'],
                        output='screen')

    # load_joint_state_broadcaster = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #          'joint_state_broadcaster'],
    #     output='screen'
    # )

    # load_joint_effort_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #          'arm_controller'],
    #     output='screen'
    # )

    # Gz - ROS Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # control world
            '/world/default/control@ros_gz_interfaces/srv/ControlWorld',
        ],
        output='screen'
    )
    
    moveit_config = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('moveit_6r'), 'launch', 'demo.launch.py'
                    )
            ),
        )

    return LaunchDescription([

        gazebo,
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=gz_spawn_entity,
        #         on_exit=[load_joint_state_broadcaster],
        #     )
        # ),

        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=load_joint_state_broadcaster,
        #         on_exit=[load_joint_effort_controller],
        #     )
        # ),

        gz_spawn_entity,

        # load_joint_state_broadcaster,
        # load_joint_effort_controller,

        moveit_config,

        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),  

        ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/move_group', 'use_sim_time', 'true']
        )
    ])
