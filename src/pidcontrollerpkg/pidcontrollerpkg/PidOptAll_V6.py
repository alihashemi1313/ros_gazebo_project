

# import libraries ............................................
import numpy as np
import math          
from decimal import *
from tokenize import Double
from typing import Counter
import pylab as py
import statistics
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

import rclpy
from rclpy.node import Node
from torque_message.srv import TRMsg
from std_srvs.srv import Empty

from .pso_algorithm import Optimizer
from .PSO_CreatModel import CreatModel
from .PSO_Plotter import Plotter
from .PSO_Obs import Obstcle




#...........................................................

getcontext().prec = 8


class TrjOptimization(Node):


    def __init__(self):
        super().__init__('minimal_client_async')


        # create service and client to apply Torque 
        self.step_call_cli = self.create_client(TRMsg, 'torque_step')
        self.req = TRMsg.Request()

        # Create service and client to reset Simulation
        self.resrt = self.create_client(Empty , 'reset_simulation')

        # PID Controller gains
        self.kp = [188920.0, 217600.0, 115330.0, 1593.0, 500.0, 22.0]
        self.kd = [18.0, 53.0, 100.0, 4.0, 0.5, 0.5]
        self.ki = [10.0, 2154.0, 2000.0, 22.524, 70.0, 1.0]


    #######################################################################
    ##                      send request to Service                      ##
    #######################################################################
    def send_request(self, Th1, Th2, Th3, Th4, Th5, Th6):

        cnt = 0     # Create the counter
        
        # initialize variables 
        current_pos = np.zeros(6) # in[rad]
        error = np.zeros(6); last_error= np.zeros(6); e_integ = np.zeros(6); f = np.zeros((6,1)) # torque
        Theta_goal = [Th1, Th2, Th3, Th4, Th5, Th6]     # Controller Desierd Value 6RJ1

        
        # Call gazebo reset service
        self.reset_call = self.resrt.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self,self.reset_call)


        # Simulation LOOP
        while cnt < len(Th1) :   # total time in seconds
            
            ff = np.zeros((6,1))

            # Apply calculated Torque on Delta robot
            self.req.t1 = float(f[0][cnt])
            self.req.t2 = float(f[1][cnt])
            self.req.t3 = float(f[2][cnt])
            self.req.t4 = float(f[3][cnt])
            self.req.t5 = float(f[4][cnt])
            self.req.t6 = float(f[5][cnt])

            # Call apply Torque service
            self.future_step = self.step_call_cli.call_async(self.req)
            rclpy.spin_until_future_complete(self,self.future_step)
            res = self.future_step.result()
            response = [res.r1, res.r2, res.r3, res.r4, res.r5, res.r6]

            for i in range(6):
                
                # Calculate PID Torque 
                current_pos[i] = response[i]
                last_error[i] = error[i] 
                error[i] = Theta_goal[i][cnt] - current_pos[i]
                e_dot = (error[i] - last_error[i]) / 0.001
                e_integ[i] = e_integ[i] + (((last_error[i] + error[i])/2)*0.001)
                ff[i] = (self.kp[i] * error[i]) + (self.kd[i] * e_dot) + (self.ki[i] * e_integ[i])
            
            
            f = np.hstack((f, ff))
            cnt += 1    # Update the counter
            
    
        # ThetaD = [Theta_goal_J1, Theta_goal_J2, Theta_goal_J3, Theta_goal_J4, Theta_goal_J5, Theta_goal_J6]
        # ThetaA = [current_pos1, current_pos2, current_pos3, current_pos4, current_pos5, current_pos6]
        return f
    


    #######################################################################
    ##                      PSO First Cost function                      ##
    #######################################################################
    def CostFun1(self,pp):
        '''
        Function definition:

        * this function gives the hole population position in every iteration,
           and returns the cost for all of them. 
        * This is the First PSO Cost Function Which includes:
                1) minimization of path length
                2) Obsecle avoidanss
        '''
    
         
        CModel = CreatModel()   # Crate Object of Crate model python Class
        pi = np.pi              # pi number
        cost = np.array([])     # Create Empty Cost numpy array
        p = pp.transpose()      # Transpose 
        
        # Create a loop for all particles in population
        for i in range(len(p[:,0])):
            # recieve Poses from Model Functiom in CreatModel python Class
            time, Pose, Theta, point = CModel.Model(p[i],True)
            # print(len(Theta[0]))
            
            
            # calculate the length of the path
            dx = np.diff(Pose[0])
            dy = np.diff(Pose[1])
            dz = np.diff(Pose[2])
            L = np.sum(np.sqrt(dx**2 + dy**2 + dz**2))

            L_line = np.sum(np.sqrt((point[0][0]-point[1][0])**2 + (point[0][1]-point[1][1])**2 + (point[0][2]-point[1][2])**2))

            cc = (L/L_line)

            # Adding Obstcle1
            obs1_violation = self.obs_Violation(Theta)
            print('obs violation is: ', obs1_violation)
            print('path length cost: ', cc)

        

            Violation = obs1_violation     # Total Violation Value
            beta = 10                      # The Violation constant Gain
            cost = np.append(cost, (-1)*((L/L_line) * (1+ beta * Violation)))

        return cost
    


    
    #######################################################################
    ##                      PSO Second Cost function                     ##
    #######################################################################
    def CostFun2(self, pp):
        '''
        Function definition:

        * this function gives the hole population position in every iteration,
           and returns the cost for all of them. 
        * This is the First PSO Cost Function Which includes:
                1) minimization of path length
                2) Obsecle avoidanss
                3) Checking out angular velocity bouonds
        '''

        CModel = CreatModel()
        pi = np.pi
        cost = np.array([])
        p = pp.transpose()

        for i in range(len(p[:,0])):
            time, Pose, Theta = CModel.Model(p[i],True)
            
            # calculate the length of the path
            dx = np.diff(Pose[0])
            dy = np.diff(Pose[1])
            dz = np.diff(Pose[2])
            L = np.sum(np.sqrt(dx**2 + dy**2 + dz**2))      #3D - line

            # Adding Obstcle1
            obs1_violation = self.obs_Violation(Pose)

            dt   = 0.001

            
            # Violation for MAX angular velocity
            OmgMax = 3000 *(2*pi /60)   #rad/sec
            sum_violation_mg = 0.0

            for i in range(1, len(Theta[0])):
                
                omega1 = (Theta[0][i] - Theta[0][i-1]) /dt
                omega2 = (Theta[1][i] - Theta[1][i-1]) /dt
                omega3 = (Theta[2][i] - Theta[2][i-1]) /dt
                omega4 = (Theta[3][i] - Theta[3][i-1]) /dt
                omega5 = (Theta[4][i] - Theta[4][i-1]) /dt
                omega6 = (Theta[5][i] - Theta[5][i-1]) /dt

                sum_violation_mg +=   max(1- ((omega1-OmgMax)/OmgMax) , 0)\
                                    + max(1- ((omega2-OmgMax)/OmgMax) , 0)\
                                    + max(1- ((omega3-OmgMax)/OmgMax) , 0)\
                                    + max(1- ((omega4-OmgMax)/OmgMax) , 0)\
                                    + max(1- ((omega5-OmgMax)/OmgMax) , 0)\
                                    + max(1- ((omega6-OmgMax)/OmgMax) , 0)
            print("-------------------------------------------------------")
            print("sum_violation_mg", sum_violation_mg)
            print("L: ", L)
            Violation = obs1_violation + sum_violation_mg / len(Theta[0])

            beta = 10
            cost = np.append(cost, (-1)*(L * (1+ beta * Violation)))
        
        return cost
    

    

    def CostFun3(self,pp):

        CModel = CreatModel()
        pi = np.pi
        cost = np.array([])
        p = pp.transpose()

        for i in range(len(p[:,0])):
            time, Pose, Theta = CModel.Model(p[i],True)
            
            # calculate the length of the path
            dx = np.diff(Pose[0])
            dy = np.diff(Pose[1])
            dz = np.diff(Pose[2])
            L = np.sum(np.sqrt(dx**2 + dy**2 + dz**2))      #3D - line

            dt   = 0.001

            
            # Violation for MAX angular velocity
            OmgMax = 3000 *(2*pi /60)   #rad/sec
            sum_violation_mg = 0.0

            for i in range(1, len(Theta[0])):
                
                omega1 = (Theta[0][i] - Theta[0][i-1]) /dt
                omega2 = (Theta[1][i] - Theta[1][i-1]) /dt
                omega3 = (Theta[2][i] - Theta[2][i-1]) /dt
                omega4 = (Theta[3][i] - Theta[3][i-1]) /dt
                omega5 = (Theta[4][i] - Theta[4][i-1]) /dt
                omega6 = (Theta[5][i] - Theta[5][i-1]) /dt

                sum_violation_mg +=   max(1- ((omega1-OmgMax)/OmgMax) , 0)\
                                    + max(1- ((omega2-OmgMax)/OmgMax) , 0)\
                                    + max(1- ((omega3-OmgMax)/OmgMax) , 0)\
                                    + max(1- ((omega4-OmgMax)/OmgMax) , 0)\
                                    + max(1- ((omega5-OmgMax)/OmgMax) , 0)\
                                    + max(1- ((omega6-OmgMax)/OmgMax) , 0)
            
            fig, axs = plt.subplots(6)
            fig.suptitle('Vertically stacked subplots')
            axs[0].plot(time, Pose[0])
            axs[1].plot(time, Pose[1])
            axs[2].plot(time, Pose[2])
            axs[3].plot(time, Pose[3])
            axs[4].plot(time, Pose[4])
            axs[5].plot(time, Pose[5])
            input()
            Trq = self.send_request(Theta[0] ,Theta[1] ,Theta[2] ,Theta[3] ,Theta[4] ,Theta[5])
            
            T =   statistics.mean(Trq[0]) + statistics.mean(Trq[1]) + statistics.mean(Trq[2])\
                + statistics.mean(Trq[3]) + statistics.mean(Trq[4]) + statistics.mean(Trq[5])
            

            TrqViolation = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            max_power = [1000, 1000, 600, 200, 200, 200]        # are in [Watt]

            Rated_Trq  = [3.19, 3.19, 1.91, 0.637, 0.637, 0.637]    # Rated Torque for Motor 1 to 6
            max_Trq200 = [6.38, 6.38, 3.82, 1.274, 1.274, 1.274]    # Max Torque 200% for Motor 1 to 6
            max_Trq300 = [9.56, 9.56, 5.34, 1.912, 1.912, 1.912]    # Max Torque 300% for Motor 1 to 6


            for i in range(6):
                # find the index of the first number between Rated Torque and Max Torque 200%
                index = next((j for j, num in enumerate(Trq[i]) if Rated_Trq[i]<= num <= max_Trq200[i]), None)

                if index is not None and index+2000 < len(Trq[i]):
                    # create the following Torque numbers to check out for being between Rated Torque and Max Torque 200%
                    subsequent_num = Trq[i][index+1 : index+2001]
                
                
                # find the index of the first number between Max Torque 200% and Max Torque 300%
                index2 = next((p for p, num in enumerate(subsequent_num) if max_Trq200[i]<= num <= max_Trq300[i]), None)
                
                # check out next 2 sec Torques for being between Rated Torque and Max Torque 200%
                if all(Rated_Trq[i]<= num <= max_Trq200[i] for num in subsequent_num):
                    subsequent_num.sort()
                    #Violation for being between Rated Torque and Max Torque 200%
                    TrqViolation[i] += (subsequent_num[-1] - Rated_Trq[i]) / (max_Trq200[i] - Rated_Trq[i])

                elif index2 is not None and index2+1000 < len(Trq[i]):
                    # create the following Torque numbers to check out for being between Max Torque 200% and Max Torque 300%
                    subsequent_num2 = Trq[i][index2+1 : index2+1001]

                    # check out next 1 sec Torques for being between Max Torque 200% and Max Torque 300%
                    if all(max_Trq200[i]<= num <= max_Trq300[i] for num in subsequent_num2):
                        subsequent_num2.sort()
                        #Violation for being between Max Torque 200% and Max Torque 300%
                        TrqViolation[i] += (subsequent_num2[-1] - max_Trq200[i]) / (max_Trq300[i] - max_Trq200[i])
                        

                    elif (max_Trq300[i]<= num for num in subsequent_num2):
                        #Violation for being over Max Torque 300%
                        TrqViolation[i] += subsequent_num2[-1] - max_Trq300[i]

                
            Violation = (sum_violation_mg / len(Theta[0])) + statistics.mean(TrqViolation)

            beta = 10
            cost = np.append(cost, (-1)*((L+T) * (1+ beta * Violation)))
        
        return cost
    

    def obs_Violation(self, Theta):

            # Creating Obstcle parameters
            Obslength = [150, 180, 1000]; centerPoint = [680, 90, 500]

            avoidOBS = Obstcle()   # Crate Object of Obstcle python Class
            Obstcle_Violation = np.array([])

            # Calculate the Violation for Obstcle
            for i in range (len(Theta[0])):

                # Create points of the path
                roboPoint = avoidOBS.robot_shape([Theta[0][i], Theta[1][i], Theta[2][i], Theta[3][i], Theta[4][i], Theta[5][i]])
                print('robot points are:', roboPoint)

                for j in range(len(roboPoint[0])):

                    # Check the path points for beeing in obstcle bounds
                    if roboPoint[0][j] >= centerPoint[0] - Obslength[0]/2 and \
                       roboPoint[0][j] <= centerPoint[0] + Obslength[0]/2 and \
                       roboPoint[1][j] >= centerPoint[1] - Obslength[1]/2 and \
                       roboPoint[1][j] <= centerPoint[1] + Obslength[1]/2 and \
                       roboPoint[2][j] >= centerPoint[2] - Obslength[2]/2 and \
                       roboPoint[2][j] <= centerPoint[2] + Obslength[2]/2 :
                    
                        # Finding the clossest point from obstcle point cloud to path point
                        mindistance, clossestPoint = avoidOBS.avoid_obstcle(Obslength, centerPoint, [roboPoint[0][j], roboPoint[1][j], roboPoint[2][j]])

                        Obstcle_Violation = np.append(Obstcle_Violation, \
                                                     (mindistance/math.sqrt((clossestPoint[0]-centerPoint[0])**2 + \
                                                                            (clossestPoint[1]-centerPoint[1])**2 + \
                                                                            (clossestPoint[2]-centerPoint[2])**2)))
                    else:
                        
                        # Not giving violation while the path point is out of obstcle
                        Obstcle_Violation = np.append(Obstcle_Violation, 0.0)


            # Make the total violation for obstcle
            Total_Obstcle_Violation = statistics.mean(Obstcle_Violation)

            return Total_Obstcle_Violation
    



def main(args=None):
    rclpy.init(args=args)  #adding this node to ROS

    # Create an object
    TrjOpt = TrjOptimization()
    
    py.rcParams.update({'font.size': 20})

    # Control parameters
    CtrlParam1 = {'w': 0.5, 'c1': 2.0, 'c2': 2.0, 'v_fct': 1,
                 'Np': 10, 'D': 18, 'max_iter': 400}
    
    CtrlParam2 = {'w': 0.5, 'c1': 2.0, 'c2': 2.0, 'v_fct': 1,
                 'Np': 10, 'D': 19, 'max_iter': 2}
    
    CtrlParam3 = {'w': 0.5, 'c1': 2.0, 'c2': 2.0, 'v_fct': 1,
                 'Np': 10, 'D': 19, 'max_iter': 10}

    xmin=-4000.0; xmax=4000.0; ymin= -4000.0; ymax=4000.0; zmin=100; zmax=1000.0 
    rmin= -2.0; rmax= 2.0; pmin= -2.0; pmax= 2.0; wmin= -2.0; wmax= 2.0; tmin= 0.2; tmax= 10

    xL = np.array([xmin,xmin,xmin, ymin,ymin,ymin, zmin,zmin,zmin, rmin,rmin,rmin, pmin,pmin,pmin, wmin,wmin,wmin])       #lower Bound
    xU = np.array([xmax,xmax,xmax, ymax,ymax,ymax, zmax,zmax,zmax, rmax,rmax,rmax, pmax,pmax,pmax, wmax,wmax,wmax])       #upper Bound

    x = np.random.rand(CtrlParam1['D'],CtrlParam1['Np'])            # Initial position of the particles
    v = np.zeros((CtrlParam1['D'],CtrlParam1['Np']))                # Initial velocity of the particles
    InitialSol = {'x': x, 'v': v}

    Opter = Optimizer()
    Model = CreatModel()
    avoidOBS = Obstcle()   # Crate Object of Obstcle python Class
    Obslength = [150-20, 180-20, 1000-20]; centerPoint = [680, 90, 500]

    # PSO1
    print("hi")
    fitness1 = TrjOpt.CostFun1
    PSO1_Res, gbest1 = Opter.PSO(xL, xU, fitness1, InitialSol, CtrlParam1)
    # print('best soluitoin PSO1 = ', gbest1)
    time1, Pose1, Theta1, point = Model.Model(gbest1, False)
        
    pc_up, pc_dwn, pc_left, pc_rigth, pc_front, pc_back = avoidOBS.obstcle_pointClud(Obslength, centerPoint, 1500)
    # r_point

    # Plot the point cloud
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.scatter(pc_up[:, 0],    pc_up[:, 1],    pc_up[:, 2],    c='b', marker='o', s=3)
    ax.scatter(pc_dwn[:, 0],   pc_dwn[:, 1],   pc_dwn[:, 2],   c='b', marker='o', s=3)
    ax.scatter(pc_left[:, 0],  pc_left[:, 1],  pc_left[:, 2],  c='b', marker='o', s=3)
    ax.scatter(pc_rigth[:, 0], pc_rigth[:, 1], pc_rigth[:, 2], c='b', marker='o', s=3)
    ax.scatter(pc_front[:, 0], pc_front[:, 1], pc_front[:, 2], c='b', marker='o', s=3)
    ax.scatter(pc_back[:, 0],  pc_back[:, 1],  pc_back[:, 2],  c='b', marker='o', s=3)



    ax.plot3D(Pose1[0], Pose1[1], Pose1[2], linewidth=4, c = 'r')

    # ax.set_aspect('equal')
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Cubic Point Cloud Obstcle')
    plt.show()
    
    fig2, axs2 = plt.subplots(6)
    fig2.suptitle('Vertically stacked subplots')

    axs2[0].plot(time1, Theta1[0])
    axs2[1].plot(time1, Theta1[1])
    axs2[2].plot(time1, Theta1[2])
    axs2[3].plot(time1, Theta1[3])
    axs2[4].plot(time1, Theta1[4])
    axs2[5].plot(time1, Theta1[5])
    plt.show()
    # plt.savefig('pso1_result.png')
    



    # fitness2 = TrjOpt.CostFun2
    # RandSol2 = {'x': np.vstack((PSO1_Res['x'], np.random.rand(1, len(PSO1_Res['x'][0])))),
    #             'v': np.vstack((PSO1_Res['v'], np.zeros((1, len(PSO1_Res['x'][0])))))
    # }
    
    # PSO2_Res, gbest2 = Opter.PSO(xL, xU, fitness2, RandSol2, CtrlParam2)
    # print('best soluitoin PSO2 = ', gbest2)
    # time2, Pose2, Theta2 = Model.Model(gbest2, True)

    # dt = 0.001
    # omega1 = [0.0]
    # omega2 = [0.0]
    # omega3 = [0.0]
    # omega4 = [0.0]
    # omega5 = [0.0]
    # omega6 = [0.0]
    
    # for i in range(1, len(Theta2[0])):
    #     omega1.append((Theta2[0][i] - Theta2[0][i-1]) /dt)
    #     omega2.append((Theta2[1][i] - Theta2[1][i-1]) /dt)
    #     omega3.append((Theta2[2][i] - Theta2[2][i-1]) /dt)
    #     omega4.append((Theta2[3][i] - Theta2[3][i-1]) /dt)
    #     omega5.append((Theta2[4][i] - Theta2[4][i-1]) /dt)
    #     omega6.append((Theta2[5][i] - Theta2[5][i-1]) /dt)

    # fig2 , axs2 = plt.subplots(6, 2)
    # fig2.suptitle(" FUM6R ")

    # axs2[0, 0].plot(time2, Theta2[0])
    # axs2[0, 0].set_title('[t, th1]')
    # axs2[0, 1].plot(time2, omega1)
    # axs2[0, 1].set_title('[t, omg1]')

    # axs2[1, 0].plot(time2, Theta2[1])
    # axs2[1, 0].set_title('[t, th2]')
    # axs2[1, 1].plot(time2, omega2)
    # axs2[1, 1].set_title('[t, omg2]')

    # axs2[2, 0].plot(time2, Theta2[2])
    # axs2[2, 0].set_title('[t, th3]')
    # axs2[2, 1].plot(time2, omega3)
    # axs2[2, 1].set_title('[t, omg3]')

    # axs2[3, 0].plot(time2, Theta2[3])
    # axs2[3, 0].set_title('[t, th4]')
    # axs2[3, 1].plot(time2, omega4)
    # axs2[3, 1].set_title('[t, omg4]')

    # axs2[4, 0].plot(time2, Theta2[4])
    # axs2[4, 0].set_title('[t, th5]')
    # axs2[4, 1].plot(time2, omega5)
    # axs2[4, 1].set_title('[t, omg5]')

    # axs2[6, 0].plot(time2, Theta2[5])
    # axs2[6, 0].set_title('[t, th6]')
    # axs2[6, 1].plot(time2, omega6)
    # axs2[6, 1].set_title('[t, omg6]')

    # plt.savefig('pso2_th_and_omg.png')

    # fig2 = plt.figure()
    # ax2 = plt.axes(projection='3d')
    # ax2.plot3D(Pose2[0], Pose2[1], Pose2[2], 'gray')
    # ax2.set_xlabel('x')
    # ax2.set_ylabel('y')
    # plt.savefig('pso2_result.png')

    # plotter.plotter2(time2, Pose2)

    # fitness3 = TrjOpt.CostFun3
    # PSO3_Res = Opter.PSO(xL, xU, fitness3, PSO2_Res, CtrlParam3)

    # print(PSO3_Res)


    # Run the inherited function from NODE class
    TrjOpt.destroy_node() 
    rclpy.shutdown()


if __name__ == '__main__':
    main()