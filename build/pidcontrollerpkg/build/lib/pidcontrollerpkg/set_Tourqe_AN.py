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

from pidcontrollerpkg.sevenseg.seven_segment_trajectory import SevenSegmentTrajectory
from .inverse_kinematics import inv_kin
from .PSO_CreatModel import CreatModel
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty

from trajectory_msgs.msg import JointTrajectoryPoint

from builtin_interfaces.msg import Duration
from rclpy.action import ActionClient

#...........................................................
getcontext().prec = 32

class MinimalClientAsync(Node):

    def __init__(self,sample_time=0.001):

        super().__init__('minimal_client_async')
        # create service and client to apply Torque 
        print("in calling service ")
        self.step_call_cli = self.create_client(TRMsg, 'torque_step')
        self.req = TRMsg.Request()
        
        self.subscription = self.create_subscription(Odometry,'/odom',self.listener_callback,10)
        self.subscription
        
        # time counter
        self.counter = Decimal(0.000)  
        _time = 5.000
        Kp = np.array([18892, 2176000, 115330, 1593, 500, 22])
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
        return self.X, self.Y, self.Z
    
    def build_trajectory(self,theta,joint_in,a_time):
            ftraj=[]
            for j in range(6):
                traj=[]
                q=[]
                traj=SevenSegmentTrajectory()
                traj.TrajectoryType = 'q2q'
                traj.q0 = Decimal(theta[j])
                traj.q1 = Decimal(joint_in[j])
                traj.v0 = Decimal(0)
                traj.v1 = Decimal(0)
                traj.v_max = Decimal(10)
                traj.a_max = Decimal(5)
                traj.j_max = Decimal(20)
                traj.dt = Decimal(0.001)
                traj.BuildTrajectory()
                for i in range(0,int(a_time*1000+1)):
                    if i<len(traj.q):
                        q.append(traj.q[i])
                    else:
                        q.append(Decimal(joint_in[j]))
                ftraj.append(q)
            ftraj=np.asarray(ftraj,dtype=float)
            print(ftraj)
            print("ok")
            print("Trajectory Build")
            return ftraj
        
    def reset_environment_request(self):

		# Everytime this function is call a request to the Reset the Environment
		# is send i.e. Move the robot to home position and change the
        print("Reseting the Environment... ")
        dt=self.sample_time
        print('Moving robot to home position...')
        self.future_step = self.step_call_cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future_step)
        self.res = self.future_step.result()
        self.counter=self.counter+Decimal(0.001)
        theta=theta=np.zeros(6)
        theta[0] = self.res.r1
        theta[1] = self.res.r2
        theta[2] = self.res.r3
        theta[3] = self.res.r4
        theta[4] = self.res.r5
        theta[5] = self.res.r6
        
        joint_in=[0,0,0,0,0,0]
        ftraj = self.build_trajectory(theta,joint_in,5)
        n=0
        _theta=np.zeros(6)
        error=np.zeros(6)
        int_error=np.zeros(6)
        _proportional=np.zeros(6)
        _derivative=np.zeros(6)
        _integral=np.zeros(6)
        set_torque=np.zeros(6)
        _last_theta=np.zeros(6)
        _last_error=np.zeros(6)
        cc=self.counter
        while self.counter <= cc+Decimal(5.000) :   # total time 
            self.future_step = self.step_call_cli.call_async(self.req)
            rclpy.spin_until_future_complete(self, self.future_step)
            self.res = self.future_step.result()
            
            _theta[0] = self.res.r1
            _theta[1] = self.res.r2
            _theta[2] = self.res.r3
            _theta[3] = self.res.r4
            _theta[4] = self.res.r5
            _theta[5] = self.res.r6
            for i in range(6):
                error[i] = float(ftraj[i][n]) - _theta[i]
                int_error[i] += ((error[i] + _last_error[i])/2)*dt
                _proportional[i] = self.Kp[i] * error[i]
                _derivative[i] = self.Kd[i] * (error[i] - _last_error[i]) / float(dt)
                _integral[i] = self.Ki[i] * self.int_error[i]
                set_torque[i] = _proportional[i] + _derivative[i] + _integral[i]
                _last_theta[i] = _theta[i]
                _last_error[i] = self.error[i]
            
            self.req.t1 = float(set_torque[0])
            self.req.t2 = float(set_torque[1])
            self.req.t3 = float(set_torque[2])
            self.req.t4 = float(set_torque[3])
            self.req.t5 = float(set_torque[4])
            self.req.t6 = float(set_torque[5])

            self._last_time = self.counter
            n = n + 1
            self.counter += Decimal(self.sample_time)
        # -----------------------------
        print("Environment Reset Success ")

    def move_up(self):
        dt = self.sample_time
        self.future_step = self.step_call_cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future_step)
        self.res = self.future_step.result()
        self.counter=self.counter+Decimal(0.001)
        theta=np.zeros(6)
        theta[0] = self.res.r1
        theta[1] = self.res.r2
        theta[2] = self.res.r3
        theta[3] = self.res.r4
        theta[4] = self.res.r5
        theta[5] = self.res.r6
        
        joint_in=[0,self.res.r2+0.36048993*pi/180,self.res.r3-1.40912154*pi/180,0,self.res.r5+1.0486316*pi/180,0]
        ftraj=self.build_trajectory(theta,joint_in,1)
        n=0
        cc=self.counter
        while self.counter <= cc+Decimal(1.000) :   # total time
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
                self.error[i] = float(ftraj[i][n]) - self._theta[i]
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
            
    # send request to Service 
    def send_request(self):
        print("moving_up")
        self.move_up()
        print("success")
        
        dt = self.sample_time
        n=0
        Cmodel = CreatModel()
        p = [720.0,750.0,760.0, -200,300,50.0,400.0, 800.0,650.0, 0,0,0,1, 0,0.5,1.5, 0.5,0.9,10]
        print("calculating inverse")
        time, Pose, theta, point = Cmodel.Model(p, True)
        self.joint_in=theta
        print("ready for actuation")
        input()
        cc = self.counter
        while self.counter <= cc+Decimal(9.999) :   # total time 
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
                self.error[i] = float(self.joint_in[i][n]*pi/180) - self._theta[i]
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
            print(float(self.set_torque[1]))

            self._last_time = self.counter
            n = n + 1
            self.counter += Decimal(self.sample_time)
            print(self.counter)
        
        print("reset?")
        input()
        self.reset_environment_request()
        print("move up?")
        input()
        self.move_up()
        
def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()
    minimal_client.send_request()
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
