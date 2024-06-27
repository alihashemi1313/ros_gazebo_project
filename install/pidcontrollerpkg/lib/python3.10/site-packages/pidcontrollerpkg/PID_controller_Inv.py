# import libraries ............................................
import sys
import signal
import time
import math
from tokenize import Double
from typing import Counter


from torque_message.srv import TRMsg
from nav_msgs.msg import Odometry


import rclpy
from rclpy.node import Node
from decimal import *
import csv
import numpy as np

#...........................................................

getcontext().prec = 8


class MinimalClientAsync(Node):


    def __init__(self):
        super().__init__('minimal_client_async')


        # create service and client to apply Torque 
        print("in calling service ")
        self.step_call_cli = self.create_client(TRMsg, 'torque_step')
        self.req = TRMsg.Request()
        
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
        

        char = [600.0 , 145.0 , 661.5] # a2 , a3 , d4

        # calculating theta1
        theta1 = float(math.atan(R[1][3]/R[0][3]))
        s1 = math.sin(theta1)

        # calculating theta3
        c3 = ((R[1][3]/s1)**2+(R[2][3]**2)-((char[2])**2)-((char[1])**2)-((char[0])**2))/(2*char[1]*char[0])
        # print("..................")
        # print(math.sqrt(1-(c3**2)))
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
        # ---------------------------- Inverse Kinematics ------------------------------------------------- #



        # initialize variables : 

        # -------------- for 6RJ1 -------------- #
        self.current_pos1 = 0.0  # theta(rad)
        self.error1_ = 0.0 
        self.last_error1= 0.0
        self.e_integ1 = 0.0
        self.f1 = 0.0   # torque
        self.Theta_goal_J1 = math.pi/6 #L[1][0] #Desierd Value 6RJ1




        # -------------- for 6RJ2 -------------- #
        self.current_pos2 = 0.0  # theta(rad)
        self.error2_ = 0.0 
        self.last_error2= 0.0
        self.e_integ2 = 0.0
        self.f2 = 0.0   # torque
        self.Theta_goal_J2 = 0.0 #L[1][1] #Desierd Value 6RJ2

        # -------------- for 6RJ3 -------------- #
        self.current_pos3 = 0.0  # theta(rad)
        self.error3_ = 0.0 
        self.last_error3= 0.0
        self.e_integ3 = 0.0
        self.f3 = 0.0   # torque
        self.Theta_goal_J3 = 0.0 #L[1][2] #Desierd Value 6RJ3


        # -------------- for 6RJ4 -------------- #
        self.current_pos4 = 0.0  # theta(rad)
        self.error4_ = 0.0 
        self.last_error4= 0.0
        self.e_integ4 = 0.0
        self.f4 = 0.0   # torque
        self.Theta_goal_J4 = 0.0 #L[1][3] #Desierd Value 6RJ4

        # -------------- for 6RJ5 -------------- #
        self.current_pos5 = 0.0  # theta(rad)
        self.current_pos5 = 0.0
        self.error5_ = 0.0 
        self.last_error5= 0.0
        self.e_integ5 = 0.0
        self.f5 = 0.0   # torque
        self.Theta_goal_J5 = 0.0 #L[1][4] #Desierd Value 6RJ5

        # -------------- for 6RJ6 -------------- #
        self.current_pos6 = 0.0  # theta(rad)
        self.current_pos6 = 0.0
        self.error6_ = 0.0 
        self.last_error6= 0.0
        self.f6 = 0.0   # torque
        self.e_integ6 = 0.0
        self.Theta_goal_J6= 0.0 #L[1][5] #Desierd Value 6RJ6



        # gains for 6RJ21
        self.k_p1 = 1000.0
        self.k_d1 = 10.0
        self.k_i1 = 100.0

        # gains for 6RJ2
        self.k_p2 = 10000.0
        self.k_d2 = 1.0
        self.k_i2 = 100.0

        # gains for 6RJ3
        self.k_p3 = 1000.0
        self.k_d3 = 10.0
        self.k_i3 = 100.0

        # gains for 6RJ4
        self.k_p4 = 1000.0
        self.k_d4 = 10.0
        self.k_i4 = 100.0

        # gains for 6RJ5
        self.k_p5 = 100.0
        self.k_d5 = 1.0
        self.k_i5 = 10.0

        # gains for 6RJ6
        self.k_p6 = 100.0
        self.k_d6 = 1.0
        self.k_i6 = 10.0
        
        # time counter
        self.counter = Decimal(0.0000)  
        
        self.subscription = self.create_subscription(Odometry , "/odom" , self.listener_callback,10)



# send request to Service 

    def send_request(self):

        #writer = csv.writer(self.file)
        while self.counter <= Decimal(30.000) :   # total time in seconds
        
                
            # Apply calculated Torque on Delta robot
            self.req.t1 = float(self.f1)
            self.req.t2 = float(self.f2)
            self.req.t3 = float(self.f3)
            self.req.t4 = float(self.f4)
            self.req.t5 = float(self.f5)
            self.req.t6 = float(self.f6)
            print("calling Service ")
            
            

            self.future_step = self.step_call_cli.call_async(self.req)
            rclpy.spin_until_future_complete(self,self.future_step)
            self.res = self.future_step.result()
            # print(self.res)


            current_pos1 = self.res.r1
            self.last_error1 = self.error1_ 
            self.error1_ = self.Theta_goal_J1 - current_pos1
            e_dot1 = (self.error1_ - self.last_error1  ) / 0.001
            self.e_integ1 = self.e_integ1 + (((self.last_error1 + self.error1_ )/2)*0.001)
            self.f1 = (self.k_p1 * self.error1_) + (self.k_d1 * e_dot1) + (self.k_i1 * self.e_integ1) 

            current_pos2 = self.res.r2
            self.last_error2 = self.error2_ 
            self.error2_ = self.Theta_goal_J2 - current_pos2
            e_dot2 = (self.error2_ - self.last_error2  ) / 0.001
            self.e_integ2 = self.e_integ2 + (((self.last_error2 + self.error2_ )/2)*0.001)
            self.f2 = (self.k_p2 * self.error2_) + (self.k_d2 * e_dot2) + (self.k_i2 * self.e_integ2) 

            current_pos3 = self.res.r3
            self.last_error3 = self.error3_ 
            self.error3_ = self.Theta_goal_J3 - current_pos3
            e_dot3 = (self.error3_ - self.last_error3  ) / 0.001
            self.e_integ3 = self.e_integ3 + (((self.last_error3 + self.error3_ )/2)*0.001)
            self.f3 = (self.k_p3 * self.error3_) + (self.k_d3 * e_dot3) + (self.k_i3 * self.e_integ3)

            current_pos4 = self.res.r4
            self.last_error4 = self.error4_ 
            self.error4_ = self.Theta_goal_J4 - current_pos4
            e_dot4 = (self.error4_ - self.last_error4  ) / 0.001
            self.e_integ4 = self.e_integ4 + (((self.last_error4 + self.error4_ )/2)*0.001)
            self.f4 = (self.k_p4 * self.error4_) + (self.k_d4 * e_dot4) + (self.k_i4 * self.e_integ4)

            current_pos5 = self.res.r5
            self.last_error5 = self.error5_ 
            self.error5_ = self.Theta_goal_J5 - current_pos5
            e_dot5 = (self.error5_ - self.last_error5  ) / 0.001
            self.e_integ5 = self.e_integ5 + (((self.last_error5 + self.error5_ )/2)*0.001)
            self.f5 = (self.k_p5 * self.error5_) + (self.k_d5 * e_dot5) + (self.k_i5 * self.e_integ5)

            current_pos6 = self.res.r6
            self.last_error6 = self.error6_ 
            self.error6_ = self.Theta_goal_J6 - current_pos6
            e_dot6 = (self.error6_ - self.last_error6  ) / 0.001
            self.e_integ6 = self.e_integ6 + (((self.last_error6 + self.error6_ )/2)*0.001)
            self.f6 = (self.k_p6 * self.error6_) + (self.k_d6 * e_dot6) + (self.k_i6 * self.e_integ6)




            # print("next_step"+str(self.counter))
            self.counter += Decimal("0.0010")      
            
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
    rclpy.init(args=args)  #adding this node to ROS

    # Create an object
    minimal_client = MinimalClientAsync()

    # Run the "send_request" of the object
    minimal_client.send_request()

    # Run the inherited function from NODE class
    minimal_client.destroy_node() 
    rclpy.shutdown()


if __name__ == '__main__':
    main()

