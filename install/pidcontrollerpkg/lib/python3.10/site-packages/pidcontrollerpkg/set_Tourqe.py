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
from pidcontrollerpkg.inverse_kinematics import inv_kin
#...........................................................
getcontext().prec = 8

class MinimalClientAsync(Node):

    def __init__(self,
                 sample_time=0.001):
                      
        super().__init__('minimal_client_async')

        # create service and client to apply Torque 
        print("in calling service ")
        self.step_call_cli = self.create_client(TRMsg, 'torque_step')
        self.req = TRMsg.Request()
        
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
        self.a_time = 1.000

        
    def build_trajectory(self):
        q1=[]
        trajectory1 = SevenSegmentTrajectory()
        trajectory1.TrajectoryType = 'q2q'
        trajectory1.InitialPosition = [0, 0, 0]
        trajectory1.FinalPosition = [-0.5, -0.5, -0.5]
        trajectory1.q0 = Decimal(self._theta[0])
        trajectory1.q1 = Decimal(self.joint_in[0])
        trajectory1.v0 = Decimal(0)
        trajectory1.v1 = Decimal(0)
        trajectory1.v_max = Decimal(10)
        trajectory1.a_max = Decimal(10)
        trajectory1.j_max = Decimal(20)
        trajectory1.dt = Decimal(0.001)
        trajectory1.BuildTrajectory()
        for i in range(0,int(self.a_time*1000+1)):
            if i<len(trajectory1.q):
                q1.append(trajectory1.q[i])
            else:
                q1.append(Decimal(self.joint_in[0]))
                    
        self.ftraj.append(q1)
        print("q1 ok")
        
        q2=[]
        trajectory2 = SevenSegmentTrajectory()
        trajectory2.TrajectoryType = 'q2q'
        trajectory2.InitialPosition = [0, 0, 0]
        trajectory2.FinalPosition = [-0.5, -0.5, -0.5]
        trajectory2.q0 = Decimal(self._theta[1])
        trajectory2.q1 = Decimal(self.joint_in[1])
        trajectory2.v0 = Decimal(0)
        trajectory2.v1 = Decimal(0)
        trajectory2.v_max = Decimal(10)
        trajectory2.a_max = Decimal(10)
        trajectory2.j_max = Decimal(20)
        trajectory2.dt = Decimal(0.001)
        trajectory2.BuildTrajectory()
        for i in range(0,int(self.a_time*1000+1)):
            if i<len(trajectory2.q):
                q2.append(trajectory2.q[i])
            else:
                q2.append(Decimal(self.joint_in[1]))
                    
        self.ftraj.append(q2)
        print("q2 ok")
        q3=[]
        trajectory3 = SevenSegmentTrajectory()
        trajectory3.TrajectoryType = 'q2q'
        trajectory3.InitialPosition = [0, 0, 0]
        trajectory3.FinalPosition = [-0.5, -0.5, -0.5]
        trajectory3.q0 = Decimal(self._theta[2])
        trajectory3.q1 = Decimal(self.joint_in[2])
        trajectory3.v0 = Decimal(0)
        trajectory3.v1 = Decimal(0)
        trajectory3.v_max = Decimal(10)
        trajectory3.a_max = Decimal(10)
        trajectory3.j_max = Decimal(20)
        trajectory3.dt = Decimal(0.001)
        trajectory3.BuildTrajectory()
        for i in range(0,int(self.a_time*1000+1)):
            if i<len(trajectory3.q):
                q3.append(trajectory3.q[i])
            else:
                q3.append(Decimal(self.joint_in[2]))
                    
        self.ftraj.append(q3)
        print("q3 ok")
        q4=[]
        trajectory4 = SevenSegmentTrajectory()
        trajectory4.TrajectoryType = 'q2q'
        trajectory4.InitialPosition = [0, 0, 0]
        trajectory4.FinalPosition = [-0.5, -0.5, -0.5]
        trajectory4.q0 = Decimal(self._theta[3])
        trajectory4.q1 = Decimal(self.joint_in[3])
        trajectory4.v0 = Decimal(0)
        trajectory4.v1 = Decimal(0)
        trajectory4.v_max = Decimal(10)
        trajectory4.a_max = Decimal(10)
        trajectory4.j_max = Decimal(20)
        trajectory4.dt = Decimal(0.001)
        trajectory4.BuildTrajectory()
        for i in range(0,int(self.a_time*1000+1)):
            if i<len(trajectory4.q):
                q4.append(trajectory4.q[i])
            else:
                q4.append(Decimal(self.joint_in[3]))
                    
        self.ftraj.append(q4)
        print("q4 ok")
        q5=[]
        trajectory5 = SevenSegmentTrajectory()
        trajectory5.TrajectoryType = 'q2q'
        trajectory5.InitialPosition = [0, 0, 0]
        trajectory5.FinalPosition = [-0.5, -0.5, -0.5]
        trajectory5.q0 = Decimal(self._theta[4])
        trajectory5.q1 = Decimal(self.joint_in[4])
        trajectory5.v0 = Decimal(0)
        trajectory5.v1 = Decimal(0)
        trajectory5.v_max = Decimal(10)
        trajectory5.a_max = Decimal(20)
        trajectory5.j_max = Decimal(30)
        trajectory5.dt = Decimal(0.001)
        trajectory5.BuildTrajectory()
        for i in range(0,int(self.a_time*1000+1)):
            if i<len(trajectory5.q):
                q5.append(trajectory5.q[i])
            else:
                q5.append(Decimal(self.joint_in[4]))
                    
        self.ftraj.append(q5)
        print("q5 ok")
        q6=[]
        trajectory6 = SevenSegmentTrajectory()
        trajectory6.TrajectoryType = 'q2q'
        trajectory6.InitialPosition = [0, 0, 0]
        trajectory6.FinalPosition = [-0.5, -0.5, -0.5]
        trajectory6.q0 = Decimal(self._theta[5])
        trajectory6.q1 = Decimal(self.joint_in[5])
        trajectory6.v0 = Decimal(0)
        trajectory6.v1 = Decimal(0)
        trajectory6.v_max = Decimal(20)
        trajectory6.a_max = Decimal(20)
        trajectory6.j_max = Decimal(30)
        trajectory6.dt = Decimal(0.001)
        trajectory6.BuildTrajectory()
        for i in range(0,int(self.a_time*1000+1)):
            if i<len(trajectory6.q):
                q6.append(trajectory6.q[i])
            else:
                q6.append(Decimal(self.joint_in[5]))
        self.ftraj.append(q6)
        print("q6 ok")
        print("Trajectory Build")
        
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
        
        self.joint_in=[self.res.r1,0.36048993*pi/180,-1.40912154*pi/180,self.res.r4,1.0486316*pi/180,self.res.r6]
        self.build_trajectory()
        n=0
        
        while self.counter <= Decimal(1.000) :   # total time 
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
                self.error[i] = float(self.ftraj[i][n]) - self._theta[i]
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

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()
    minimal_client.send_request()
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    print("6r_cli: enter 1 to start 0 to exit:")
    nump = input()
    if nump == 1:
        main()
    else:
        exit
    
