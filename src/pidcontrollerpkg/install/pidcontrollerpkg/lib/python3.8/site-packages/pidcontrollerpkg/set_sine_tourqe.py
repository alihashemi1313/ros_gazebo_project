# import libraries ............................................
import sys
import signal
import time
import math
from tokenize import Double
from typing import Counter


from torque_message.srv import TRMsg
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState


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
        
        
        # time counter
        self.counter = Decimal(0.0000)  



# send request to Service 

    def send_request(self):

        
        self.T=float(2.000)
        while self.counter <= Decimal(2.000) :   # total time 
        
                
            t=float(self.counter)
            # Apply calculated Torque on Delta robot
            self.req.t1 = math.sin(2*math.pi*t/self.T)
            self.req.t2 = math.sin(2*math.pi*t/self.T)
            self.req.t3 = math.sin(2*math.pi*t/self.T)
            self.req.t4 = math.sin(2*math.pi*t/self.T)
            self.req.t5 = math.sin(2*math.pi*t/self.T)
            self.req.t6 = math.sin(2*math.pi*t/self.T)
            print("calling Service ")
            
		
            self.future_step = self.step_call_cli.call_async(self.req)
            print("next_step"+str(self.counter))
            self.counter += Decimal("0.0010")      
            
      




def main(args=None):
    rclpy.init(args=args)

    
    minimal_client = MinimalClientAsync()
    minimal_client.send_request()
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
