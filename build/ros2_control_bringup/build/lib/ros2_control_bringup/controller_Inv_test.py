import sys
import signal
import time
import math
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory , JointTrajectoryPoint
from nav_msgs.msg import Odometry

import numpy as np


class Trajectory_publisher(Node):

    def __init__(self):

        super().__init__('trajectory_publsiher_node')
        publish_topic = "/joint_trajectory_controller/joint_trajectory"
        self.trajectory_publihser = self.create_publisher(JointTrajectory,publish_topic, 10)
        timer_period = 2
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.subscription = self.create_subscription(Odometry , "/odom" , self.listener_callback,10)
        self.joints = ['6RJ1','6RJ2','6RJ3','6RJ4','6RJ5','6RJ6']

        # ---------------------------- Inverse Kinematics ------------------------------------------------- #

        pi = math.pi

        # Coverting x , y , z , roll , pith , yaw to Transferfunction
        p= [-572.8758,-330.75,745.0,0.3,0.3,0.3]  #[x(mm),y(mm),z(mm),roll(rad),pitch(rad),yaw(rad)]

        Rz = [[math.cos(p[5]),-math.sin(p[5]),0,0],[math.sin(p[5]),math.cos(p[5]),0,0],[0,0,1,0],[0,0,0,1]]

        Ry = [[math.cos(p[4]),0,math.sin(p[4]),0],[0,1,0,0],[-math.sin(p[4]),0,math.cos(p[4]),0],[0,0,0,1]]

        Rx = [[1,0,0,0],[0,math.cos(p[3]),-math.sin(p[3]),0],[0,math.sin(p[3]),math.cos(p[3]),0],[0,0,0,1]]

        R= np.dot(Rz,np.dot(Ry,Rx))
        R[0][3]=p[0]
        R[1][3]=p[1]
        R[2][3]=p[2]
        print(R)

        # r11 = float(-0.75)
        # r12 = float(-0.5)
        # r13 = float(-0.433)
        # r14 = float(-572.8758)
        # r21 = float(-0.433)
        # r22 = float(0.866)
        # r23 = float(-0.25)
        # r24 = float(-330.75)
        # r31 = float(0.5)
        # r32 = float(0.0)
        # r33 = float(-0.866)
        # r34 = float(745.0)
        # r41 = float(0.0)
        # r42 = float(0.0)
        # r43 = float(0.0)
        # r44 = float(1.0)
        
        
        

        char = [600.0 , 145.0 , 661.5] # a2 , a3 , d4

        # calculating theta1
        theta1 = float(math.atan(R[1][3]/R[0][3]))
        s1 = math.sin(theta1)

        # calculating theta3
        c3 = ((R[1][3]/s1)**2+(R[2][3]**2)-((char[2])**2)-((char[1])**2)-((char[0])**2))/(2*char[1]*char[0])
        print("..................")
        print(math.sqrt(1-(c3**2)))
        s3_ = float(math.sqrt(1-(c3**2)))
        s3 = [s3_ , -s3_]
        thetaa_3 = [float(math.atan(s3[0]/(c3+0.01))) , float(math.atan(s3[1]/(c3+0.01)))]


        L = [[None,None,None,None,None,None]]

        for i in range(2):
            theta3 = thetaa_3[i]
            bb = char[1]+char[0]*float(math.cos(theta3))*1
            cc = char[2]-char[0]*float(math.sin(theta3))*1
            s23 = -(-bb*R[2][3]+cc*(math.cos(theta1)*R[0][3]+math.sin(theta1)*R[1][3]))/((math.cos(theta1)*R[0][3]+math.sin(theta1)*R[1][3])**2+(R[2][3] **2))
            c23 = (bb-s23*R[2][3])/(math.cos(theta1)*R[0][3]+math.sin(theta1)*R[1][3])
            theta2 = math.atan(s23/c23)-theta3
            c5 = -s23*math.cos(theta1)*R[0][2]-s23*math.sin(theta1)*R[1][2]+c23*R[2][2]
            s5_ = math.sqrt(1-(c5**2))
            s5 = [s5_ , -s5_]
            thetaa5 = [math.atan(s5[0]/c5) , math.atan(s5[1]/c5)]

            for j in range(2):
                theta5 = thetaa5[j]
                k1 = c23*math.cos(theta1)*R[0][0]-s23*math.sin(theta1)*R[1][0]+c23*R[2][0]
                k2 = math.sin(theta1)*R[0][2]-math.cos(theta1)*R[1][2]
                theta4 = math.atan(-k2/k1)
                k3 = -s23*math.cos(theta1)*R[0][0]-s23*math.sin(theta1)*R[1][0]+c23*R[2][0]
                k4 = -s23*math.cos(theta1)*R[0][1]-s23*math.sin(theta1)*R[1][1]+c23*R[2][1]
                theta6 = math.atan(-k4/k3)
        
                newrow1 = [theta1, theta2, theta3, theta4, theta5, theta6]
                L = np.vstack([L, newrow1])
                newrow2 = [theta1+pi, theta2+pi, theta3+pi, theta4+pi, theta5+pi, theta6+pi]
                L = np.vstack([L, newrow2])
                # print(type(L))
        # ---------------------------- Inverse Kinematics ------------------------------------------------- #    
        #self.goal_positions = [L[1][0],L[1][1],L[1][2],L[1][3],L[1][4],L[1][5]]
        self.goal_positions = [-pi/6,0.0,0.0,0.0,0.0,0.0]
    
    # def listener_callback(self,msg):
    #     print("Pose x = ",msg.pose.pose.position.x)
    #     print("Pose y = ",msg.pose.pose.position.y)
    #     print("Pose z = ",msg.pose.pose.position.z)


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
        self.VX = msg.twist.twist.linear.x
        self.VY = msg.twist.twist.linear.y
        self.VZ = msg.twist.twist.linear.z


        print("--------------------------------------")
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
