# import libraries ............................................
import sys
import signal
import time
import math
from math import pi 
from tokenize import Double
from typing import Counter

from torque_message.srv import TRMsg
import rclpy
from rclpy.node import Node
from decimal import *
import csv
import numpy as np

import ikpy
from ikpy.chain import Chain

from pidcontrollerpkg.sevenseg.seven_segment_trajectory import SevenSegmentTrajectory
from pidcontrollerpkg.PSO_CreatModel import CreatModel
from pidcontrollerpkg.inverse_kinematics import inv_kin
from nav_msgs.msg import Odometry
#...........................................................
getcontext().prec = 8

class MinimalClientAsync(Node):

    def __init__(self,sample_time=0.001):
                      
        super().__init__('minimal_client_async')
        # create service and client to apply Torque 
        print("in calling service ")
        self.step_call_cli = self.create_client(TRMsg, 'torque_step')
        self.req = TRMsg.Request()
        self.subscription = self.create_subscription(Odometry,'/odom',self.listener_callback,10)

        # time counter
        self.counter = Decimal(0.000)  
        _time = 5.000
        Kp = np.array([188920, 2176000, 115330, 1593, 500, 22])
        Kd = np.array([18, 53.21, 100, 4.06, 0.5, 1])
        Ki = np.array([10, 2154, 2000, 22.524, 70, 1])

        #Kp = np.array([500, 10100, 100, 50, 50, 30])
        #Kd = np.array([3, 10, 4, 2, 1, 0.5])
        #Ki = np.array([0.01, 0.1, 0.01, 0.01, 0.01, 0.01])
        self.Kp, self.Kd, self.Ki = Kp , Kd , Ki
        self.sample_time = sample_time

        self._proportional = np.zeros(6)
        self._integral = np.zeros(6)
        self._derivative = np.zeros(6)

        self._last_time = self.counter
        self._last_theta = np.zeros(6)
        self._last_error = np.zeros(6)
        self.int_error = np.zeros(6)
        self.int_sum = np.zeros(6)
        self.int_lastsum = np.zeros(6)
        self._theta = np.zeros(6)
        self.error = np.zeros(6)
        self._last_error = np.zeros(6)
        self.d_theta = np.zeros(6)
        self.set_torque = np.zeros(6)
        self.ftraj = []
        self.a_time = 5.000
        
    def listener_callback(self, msg):
        self.X = msg.pose.pose.position.x
        self.Y = msg.pose.pose.position.y
        self.Z = msg.pose.pose.position.z
        self.VX = msg.pose.pose.orientation.x
        self.VY = msg.pose.pose.orientation.y
        self.VZ = msg.pose.pose.orientation.z

    # send request to Service 
    def send_request(self):
        dt = self.sample_time
        self.future_step = self.step_call_cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future_step)
        
        self.res = self.future_step.result()
    
        self._theta[0] = self.res.r1
        self._theta[1] = self.res.r2
        self._theta[2] = self.res.r3
        self._theta[3] = self.res.r4
        self._theta[4] = self.res.r5
        self._theta[5] = self.res.r6

        n=0
        
        while self.counter0 <= Decimal(5.000) :   # total time 
            self.future_step = self.step_call_cli.call_async(self.req)
            rclpy.spin_until_future_complete(self, self.future_step)
            self.res = self.future_step.result()
            
            self._theta[0] = self.res.r1
            self._theta[1] = self.res.r2
            self._theta[2] = self.res.r3
            self._theta[3] = self.res.r4
            self._theta[4] = self.res.r5
            self._theta[5] = self.res.r6
            
            for i in range(6):
                self.error[i] = float(self.joint_in[n][i]) - self._theta[i]
                self.int_error[i] += ((self.error[i] + self._last_error[i])/2)*dt
                self._proportional[i] = self.Kp[i] * self.error[i]
                self._derivative[i] = self.Kd[i] * (self.error[i] - self._last_error[i]) / float(dt)
                self._integral[i] = self.Ki[i] * self.int_error[i]
                self.set_torque[i] = self._proportional[i] + self._derivative[i] + self._integral[i]
                self._last_theta[i] = self._theta[i]
                self._last_error[i] = self.error[i]
            
            self.req.t1 = float(self.set_torque[0])
            self.req.t2 = float(self.set_torque[1])
            self.req.t3 = float(self.set_torque[2])
            self.req.t4 = float(self.set_torque[3])
            self.req.t5 = float(self.set_torque[4])
            self.req.t6 = float(self.set_torque[5])

            self._last_time = self.counter
            n = n + 1
            self.counter += Decimal(self.sample_time)

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(Odometry,'/odom',self.listener_callback,10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.X = msg.pose.pose.position.x
        self.Y = msg.pose.pose.position.y
        self.Z = msg.pose.pose.position.z
        self.roll = msg.pose.pose.orientation.x
        self.pitch = msg.pose.pose.orientation.y
        self.yaw = msg.pose.pose.orientation.z

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()
    minimal_client.send_request()
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
