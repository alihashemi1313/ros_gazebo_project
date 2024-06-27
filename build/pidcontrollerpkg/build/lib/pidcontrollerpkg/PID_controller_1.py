# import libraries ............................................
import sys
import signal
import time
import math
from tokenize import Double
from typing import Counter


from torque_message.srv import TRMsg
# from std_srvs.srv import Empty
# from nav_msgs.msg import Odometry
# from sensor_msgs.msg import JointState


import rclpy
from rclpy.node import Node
from decimal import *
import csv
import numpy as nps

#...........................................................

getcontext().prec = 8


class MinimalClientAsync(Node):


    def __init__(self):
        super().__init__('minimal_client_async')


        # create service and client to apply Torque 
        print("in calling service ")
        self.step_call_cli = self.create_client(TRMsg, 'torque_step')
        #while not self.step_call_cli.wait_for_service(timeout_sec=1.0):
            #self.get_logger().info('service not available, waiting again... torque step')
        #self.reset_call_cli = self.create_client(Empty , 'reset_simulation')
        #self.future = self.reset_call_cli.call_async(Empty.Request())
        #rclpy.spin_until_future_complete(self,self.future)
        self.req = TRMsg.Request()
        



# initialize variables : 

        # -------------- for 6RJ1 -------------- #
        self.current_pos1 = 0.0  # theta(rad)
        self.error1_ = 0.0 
        self.last_error1= 0.0
        self.e_integ1 = 0.0
        self.f1 = 0.0   # torque
        self.Theta_goal_J1 = 0.5 #Desierd Value 6RJ1




        # -------------- for 6RJ2 -------------- #
        self.current_pos2 = 0.0  # theta(rad)
        self.error2_ = 0.0 
        self.last_error2= 0.0
        self.e_integ2 = 0.0
        self.f2 = 0.0   # torque
        self.Theta_goal_J2 = 0.5 #Desierd Value 6RJ2

        # -------------- for 6RJ3 -------------- #
        self.current_pos3 = 0.0  # theta(rad)
        self.error3_ = 0.0 
        self.last_error3= 0.0
        self.e_integ3 = 0.0
        self.f3 = 0.0   # torque
        self.Theta_goal_J3 = 0.5 #Desierd Value 6RJ3


        # -------------- for 6RJ4 -------------- #
        self.current_pos4 = 0.0  # theta(rad)
        self.error4_ = 0.0 
        self.last_error4= 0.0
        self.e_integ4 = 0.0
        self.f4 = 0.0   # torque
        self.Theta_goal_J4 = 0.5 #Desierd Value 6RJ4

        # -------------- for 6RJ5 -------------- #
        self.current_pos5 = 0.0  # theta(rad)
        self.current_pos5 = 0.0
        self.error5_ = 0.0 
        self.last_error5= 0.0
        self.e_integ5 = 0.0
        self.f5 = 0.0   # torque
        self.Theta_goal_J5 = 0.5 #Desierd Value 6RJ5

        # -------------- for 6RJ6 -------------- #
        self.current_pos6 = 0.0  # theta(rad)
        self.current_pos6 = 0.0
        self.error6_ = 0.0 
        self.last_error6= 0.0
        self.e_integ6 = 0.0
        self.Theta_goal_J6= 0.5 #Desierd Value 6RJ6


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



        # create subscribers for getting Joint positions from JointState plugin in Gazebo 
        # self.joint_state_subscriber1 = self.create_subscription(JointState , "/J1space/joint_states" , self.JointState_6RJ1 , 10)
        # self.joint_state_subscriber2 = self.create_subscription(JointState , "/J2space/joint_states" , self.JointState_6RJ2 , 10)
        # self.joint_state_subscriber3 = self.create_subscription(JointState , "/J3space/joint_states" , self.JointState_6RJ3 , 10)
        # self.joint_state_subscriber4 = self.create_subscription(JointState , "/J4space/joint_states" , self.JointState_6RJ4 , 10)
        # self.joint_state_subscriber5 = self.create_subscription(JointState , "/J5space/joint_states" , self.JointState_6RJ5 , 10)
        # self.joint_state_subscriber6 = self.create_subscription(JointState , "/J6space/joint_states" , self.JointState_6RJ6 , 10)


        #f dec
        self.f1 = 0 
        self.f2 = 0 
        self.f3 = 0 
        self.f4 = 0 
        self.f5 = 0 
        self.f6 = 0




# Calculate Torque using PID Algorithm in JointState Functions for each joint


#comment bcs we added to plugin as a response as r1 and r2 and r3 and r4 and r5 and r6 
#check res srv message 

    # def JointState_6RJ1(self , msg ) : 
    #     self.current_pos1 = msg.position[0]
    #     self.last_error1 = self.error1_ 
    #     self.error1_ = self.Theta_goal_J1 - self.current_pos1
    #     e_dot = (self.error1_ - self.last_error1  ) / 0.001
    #     self.e_integ1 = self.e_integ1 + (((self.last_error1 + self.error1_ )/2)*0.001)
    #     self.f1 = (self.k_p1 * self.error1_) + (self.k_d1 * e_dot) + (self.k_i1 * self.e_integ1) 
    #     print("joint1")



    # def JointState_6RJ2(self , msg ) : 
    #     self.current_pos2 = msg.position[0]
    #     self.last_error2 = self.error2_ 
    #     self.error2_ = self.Theta_goal_J2 - self.current_pos2
    #     e_dot2 = (self.error2_ - self.last_error2  ) / 0.001
    #     self.e_integ2 = self.e_integ2 + (((self.last_error2 + self.error2_ )/2)*0.001)
    #     self.f2 = (self.k_p2 * self.last_error2) + (self.k_d2 * e_dot2) + (self.k_i2 * self.e_integ2) 
    #     print("joint2 ")



    # def JointState_6RJ3(self , msg ) : 
    #     self.current_pos3 = msg.position[0]
    #     self.last_error3 = self.error3_ 
    #     self.error3_ = self.Theta_goal_J3 - self.current_pos3
    #     e_dot3 = (self.error3_ - self.last_error3  ) / 0.001
    #     self.e_integ3 = self.e_integ3 + (((self.last_error3 + self.error3_ )/2)*0.001)
    #     self.f3 = (self.k_p3 * self.last_error3) + (self.k_d3 * e_dot3) + (self.k_i3 * self.e_integ3) 
    #     print("joint3")




    # def JointState_6RJ4(self , msg ) : 
    #     self.current_pos4 = msg.position[0]
    #     self.last_error4 = self.error4_ 
    #     self.error4_ = self.Theta_goal_J4 - self.current_pos4
    #     e_dot4 = (self.error4_ - self.last_error4  ) / 0.001
    #     self.e_integ4 = self.e_integ4 + (((self.last_error4 + self.error4_ )/2)*0.001)
    #     self.f4 = (self.k_p4 * self.last_error4) + (self.k_d4 * e_dot4) + (self.k_i4 * self.e_integ4) 
    #     print("joint4")




    # def JointState_6RJ5(self , msg ) : 
    #     self.current_pos5 = msg.position[0]
    #     self.last_error5 = self.error5_ 
    #     self.error5_ = self.Theta_goal_J5 - self.current_pos5
    #     e_dot5 = (self.error5_ - self.last_error5  ) / 0.001
    #     self.e_integ5 = self.e_integ5 + (((self.last_error5 + self.error5_ )/2)*0.001)
    #     self.f5 = (self.k_p5 * self.last_error5) + (self.k_d5 * e_dot5) + (self.k_i5 * self.e_integ5) 
    #     print("joint5")



    # def JointState_6RJ6(self , msg ) : 
    #     self.current_pos6 = msg.position[0]
    #     self.last_error6 = self.error6_ 
    #     self.error6_ = self.Theta_goal_J6 - self.current_pos6
    #     e_dot6 = (self.error6_ - self.last_error6  ) / 0.001
    #     self.e_integ6 = self.e_integ6 + (((self.last_error6 + self.error6_ )/2)*0.001)
    #     self.f6 = (self.k_p6 * self.last_error6) + (self.k_d6 * e_dot6) + (self.k_i6 * self.e_integ6)  
    #     print("joint6")






    

# send request to Service 

    def send_request(self):

        #writer = csv.writer(self.file)
        while self.counter <= Decimal(5.000) :   # total time in seconds
        
                
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
            print(self.res)


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




            print("next_step"+str(self.counter))
            self.counter += Decimal("0.0010")      
            
      




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
