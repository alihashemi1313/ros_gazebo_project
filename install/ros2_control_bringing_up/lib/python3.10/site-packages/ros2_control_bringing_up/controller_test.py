import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory , JointTrajectoryPoint
import math
import numpy as np
from nav_msgs.msg import Odometry

class Trajectory_publisher(Node):

    def __init__(self):
        super().__init__('trajectory_publsiher_node')
        self.subscription = self.create_subscription(Odometry,'/odom',self.listener_callback,10)
        self.subscription  # prevent unused variable warning
        publish_topic = "/joint_trajectory_controller/joint_trajectory"
        self.trajectory_publihser = self.create_publisher(JointTrajectory,publish_topic, 10)
        timer_period = 2
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.joints = ['6RJ1','6RJ2','6RJ3','6RJ4','6RJ5','6RJ6']   
        self.goal_positions = [0.0,0.0,0.0,0.0,0.0,0.0]
        

    def timer_callback(self):
        bazu_trajectory_msg = JointTrajectory()
        bazu_trajectory_msg.joint_names = self.joints
        point = JointTrajectoryPoint()
        point.positions = self.goal_positions
        point.time_from_start = Duration(sec=2)
        bazu_trajectory_msg.points.append(point)
        self.trajectory_publihser.publish(bazu_trajectory_msg)
        
    def listener_callback(self, msg):
        self.X = msg.pose.pose.position.x
        self.Y = msg.pose.pose.position.y
        self.Z = msg.pose.pose.position.z
        self.VX = msg.pose.pose.orientation.x
        self.VY = msg.pose.pose.orientation.y
        self.VZ = msg.pose.pose.orientation.z


        print('x= ',self.X,'and vx=',self.VX)
        print('y= ',self.Y,'and vy=',self.VY)
        print('z= ',self.Z,'and vz=',self.VZ)
        
def main(args=None):
    rclpy.init(args=args)
    joint_trajectory_object = Trajectory_publisher()

    rclpy.spin(joint_trajectory_object)
    joint_trajectory_object.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#https://github.com/ros-controls/gz_ros2_control