# import libraries ............................................
import threading
import sys
import signal
import time
import math
from tokenize import Double
from typing import Counter
from pidcontrollerpkg.sevenseg.seven_segment_trajectory import SevenSegmentTrajectory

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import rclpy
from rclpy.node import Node
from rclpy.time import Duration
from decimal import *
import numpy as np
from scipy.interpolate import InterpolatedUnivariateSpline
from ros_gz_interfaces.srv import ControlWorld
#...........................................................
getcontext().prec = 8

class MinimalClientAsync(Node):


    def __init__(self):
        super().__init__('minimal_client_async')
        self.joint_position = self.create_subscription(JointState, '/joint_states', self.get_pose, 10)
        self.joint_trajectory_publisher = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

        # self.control_cli = self.create_client(ControlWorld,'/world/default/control')
        # while not self.control_cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service %s not available, waiting again...')

        # self.control_cli.wait_for_service()
        
        timer_period = 0.5 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.target_positions = [0.0 ,0.0,0.0,0.0,0.0,0.0]
        self.joint_names = ["6RJ1","6RJ2","6RJ3","6RJ4","6RJ5","6RJ6"]
    def pause(self,step=0, is_wait=False):
        req = ControlWorld.Request()
        req.world_control.pause = True
        req.world_control.multi_step = step
        srv_call = self.control_cli.call_async(req)
        if is_wait:
            #wait
            while rclpy.ok(): 
                if srv_call.done():
                    break
            #result
            return srv_call.result().success
        return True
    
    def resume(self,is_wait=False):
        req = ControlWorld.Request()
        req.world_control.pause = False
        srv_call = self.control_cli.call_async(req)
        if is_wait:
            #wait
            while rclpy.ok(): 
                if srv_call.done():
                    break
            #result
            return srv_call.result().success
        return True

    def timer_callback(self):

        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        # if self.i%1000 == 0:
        #     point.positions = [0.0 ,0.0,0.0,0.0,0.0,0.0]
        # else:
        point.positions = [0.5 ,0.5,0.0,0.0,0.0,0.0]
        self.i+=1
        point.time_from_start = Duration(seconds=1).to_msg()
        msg.points.append(point)
        self.joint_trajectory_publisher.publish(msg)
    
    def get_pose(self,msg):
        j2 = msg.position[0]
        j4 = msg.position[1]
        j1 = msg.position[2]
        j5 = msg.position[3]
        j3 = msg.position[4]
        j6 = msg.position[5]
        self.joint_pose = msg.position
        self.joint_name = msg.name

def main(args=None):
    rclpy.init(args=args)  #adding this node to ROS

    send_torque = MinimalClientAsync()
    # time.sleep(1)
    # send_torque.pause(1,False)
    # time.sleep(2)
    # send_torque.pause(1,False)
    # time.sleep(2)
    rclpy.spin(send_torque)

    # Run the inherited function from NODE class
    send_torque.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()