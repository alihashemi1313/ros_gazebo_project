import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory , JointTrajectoryPoint
import math
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseStamped
from pymoveit2 import MoveIt2
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile
class Trajectory_publisher(Node):

    def __init__(self):
        super().__init__('trajectory_publsiher_node')

        # Create callback group that allows execution of callbacks in parallel without restrictions
        self._callback_group = ReentrantCallbackGroup()

        # Create MoveIt 2 interface
        self._moveit2 = MoveIt2(
            node=self,
            joint_names = ["6RJ1","6RJ2","6RJ3","6RJ4","6RJ5","6RJ6"],
            base_link_name = "6RL0",
            end_effector_name = "6RL6",
            group_name = "arm",
            execute_via_moveit = True,
            callback_group = self._callback_group,
        )

        # Use upper joint velocity and acceleration limits
        self._moveit2.max_velocity = 15.0
        self._moveit2.max_acceleration = 10.0

        # Create a subscriber for target pose
        self._previous_target_pose = Pose()
        self.create_subscription(
            msg_type=PoseStamped,
            topic="/target_pose",
            callback=self.target_pose_callback,
            qos_profile=QoSProfile(depth=1),
            callback_group=self._callback_group,
        )



    # def timer_callback(self):

        
    # def listener_callback(self, msg):
  
def main(args=None):
    rclpy.init(args=args)
    joint_trajectory_object = Trajectory_publisher()
    
    rclpy.spin(joint_trajectory_object)
    joint_trajectory_object.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#https://github.com/ros-controls/gz_ros2_control