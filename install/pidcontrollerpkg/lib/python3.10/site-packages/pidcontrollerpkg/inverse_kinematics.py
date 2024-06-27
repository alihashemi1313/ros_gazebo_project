import math
import numpy as np
import matplotlib.pyplot as plt
import csv
from decimal import *
import pandas as pd
from scipy.spatial.transform import Rotation
class inv_kin():

    def __init__(self):
        print("object created")


    def inv_k(self,x,y,z,roll,pitch,yaw,t1,t2,t3,t4,t5,t6):
        
        L1 = 389.5; L2 = 0.0; L3 = 600.0; L4 = 200.0; L5 = 685.5; # in [mm]
        pi = math.pi
        QT = [math.cos(0)    ,0, 0, 0     ,0 ,0 ,0 ,0]
        QBase = [math.cos(0)    ,0, 0, 0    ,0 ,0 ,0 ,0]
        QTool = [math.cos(0)    ,0 ,0 ,0    ,0 ,0 ,0 ,0]
        Qend = self.angle2quat(yaw,pitch,roll)
        MT = np.append(Qend ,[0 ,x ,y ,z])
        MT = MT.tolist()

        CurPos = [t1 ,t2 ,t3 ,t4 ,t5 ,t6]
        QB = self.DQinv(QBase)
        ## Inverse Kinematic
        M8 = (self.DQmultiply(QB, MT))  #Calculate the Parameters of DualQuat of M7 (End Effector)
        M7 = (self.DQmultiply(M8, self.DQinv(QTool)))  #Calculate the Parameters of DualQuat of M7 (End Effector)
        M6 = (self.DQmultiply(M7, self.DQinv(QT)))     #Calculate the Parameters of DualQuat of M6 (Wrist)

        
        Qg1=M6[0]; Qg2=M6[1]; Qg3=M6[2]; Qg4=M6[3]
        Qg5=M6[4]; Qg6=M6[5]; Qg7=M6[6]; Qg8=M6[7]
        Qg = [Qg1 ,Qg2 ,Qg3 ,Qg4 ,Qg5 ,Qg6 ,Qg7 ,Qg8]

        InK = np.zeros((8,6))
        DInK = np.zeros((8,6))
        Teta1 = np.zeros(2)
        Teta2 = np.zeros((2,2))
        Teta3 = np.zeros((2,2))
        Teta4 = np.zeros((2,2,2))
        Teta5 = np.zeros((2,2,2))
        Teta6 = np.zeros((2,2,2))
        N4 = np.zeros(4)
        ##### checked!

        for i in range(1, 3):
            # Theta1
            # Modificatin of Shoulder Singularity
            if (M6[5])**2 + (M6[6])**2 < (0.1)**2:

                Teta1[i-1] = CurPos[0]*pi/180 - (i-1)*pi
                InK[4*i-4:4*i, 0] = Teta1[i-1]*180/pi
                t1=Teta1[i-1]
                print('Shoulder')

            else:
                a=Qg6; b=Qg7; c=0.0; 

                Teta1[i-1] = math.atan2(b,a) - math.atan2(c ,(-2*i+3) * math.sqrt(a**2+b**2-c**2))
                
                InK[4*i-4:4*i,0] = Teta1[i-1]*180/pi

                t1=Teta1[i-1]


            for j in range(1, 3):
                #Teta2
                a= (-2)*L3*Qg6*math.cos(t1) - 2*L3*Qg7*math.sin(t1) + 2*L2*L3 
                
                b= 2*L3*Qg8 - 2*L1*L3
                
                c= Qg6**2 *(math.cos(t1))**2 - Qg7**2*(math.cos(t1))**2 + L1**2 + L2**2 + L3**2 + Qg7**2 + Qg8**2 - 2*L1*Qg8 + Qg6*Qg7*math.sin(2*t1)  - 2*L2*Qg6*math.cos(t1) - 2*L2*Qg7*math.sin(t1) - L4**2 - L5**2
                
                Teta2[i-1][j-1] = math.atan2(b,a)- math.atan2(c, (2*j-3)* math.sqrt(abs(a**2+b**2-c**2)))

                InK[4*i+2*j-6 : 4*i+2*j-4,1] =Teta2[i-1][j-1]*180/pi
               
                t2=Teta2[i-1][j-1]
                H=a**2+b**2-c**2
                # Verification of Secend Branch of Teta1
                if (a**2+b**2-c**2)<0:
                    H=1
                
                # Teta3
                a=  2*L3*L5
                b=  2*L3*L4
                c= (Qg6*math.cos(t1) + Qg7*math.sin(t1) - L2)**2 + (Qg8 - L1)**2 - L3**2 - L4**2 - L5**2
                Teta3[i-1][j-1] = math.atan2(b,a) - math.atan2(c, (-2*j+3)*math.sqrt(abs(a**2+b**2-c**2)))
                
                InK[4*i+2*j-6 : 4*i+2*j-4,2] = Teta3[i-1][j-1]*180/pi
                t3=Teta3[i-1][j-1]

                for k in range(1, 3):

                    N4[0]=np.cos(t3/2)*(np.cos(t2/2)*(Qg1*np.cos(t1/2) + Qg4*np.sin(t1/2)) + np.sin(t2/2)*(Qg3*np.cos(t1/2) - Qg2*np.sin(t1/2))) + np.sin(t3/2)*(np.cos(t2/2)*(Qg3*np.cos(t1/2) - Qg2*np.sin(t1/2)) - np.sin(t2/2)*(Qg1*np.cos(t1/2) + Qg4*np.sin(t1/2)))
                    N4[1]= np.cos(t3/2)*(np.cos(t2/2)*(Qg2*np.cos(t1/2) + Qg3*np.sin(t1/2)) - np.sin(t2/2)*(Qg4*np.cos(t1/2) - Qg1*np.sin(t1/2))) - np.sin(t3/2)*(np.cos(t2/2)*(Qg4*np.cos(t1/2) - Qg1*np.sin(t1/2)) + np.sin(t2/2)*(Qg2*np.cos(t1/2) + Qg3*np.sin(t1/2)))
                    N4[2]=np.cos(t3/2)*(np.cos(t2/2)*(Qg3*np.cos(t1/2) - Qg2*np.sin(t1/2)) - np.sin(t2/2)*(Qg1*np.cos(t1/2) + Qg4*np.sin(t1/2))) - np.sin(t3/2)*(np.cos(t2/2)*(Qg1*np.cos(t1/2) + Qg4*np.sin(t1/2)) + np.sin(t2/2)*(Qg3*np.cos(t1/2) - Qg2*np.sin(t1/2)))
                    N4[3]=np.cos(t3/2)*(np.cos(t2/2)*(Qg4*np.cos(t1/2) - Qg1*np.sin(t1/2)) + np.sin(t2/2)*(Qg2*np.cos(t1/2) + Qg3*np.sin(t1/2))) + np.sin(t3/2)*(np.cos(t2/2)*(Qg2*np.cos(t1/2) + Qg3*np.sin(t1/2)) - np.sin(t2/2)*(Qg4*np.cos(t1/2) - Qg1*np.sin(t1/2)))
                    a= 2*math.atan2(N4[1],N4[0])
                    b= 2*math.atan2(N4[3],N4[2])

                    #Teta4
                    Teta4[i-1][j-1][k-1]=(a+b)/2 - (k-1)*pi
                    InK[4*i+2*j+k-7,3]=Teta4[i-1][j-1][k-1]*180/pi
                    t4=Teta4[i-1][j-1][k-1]

                    #Teta5
                    Teta5[i-1][j-1][k-1]=(2*math.atan2(math.sqrt((N4[2])**2+(N4[3])**2),math.sqrt((N4[0])**2+(N4[1])**2)))*(-2*k+3)
                    InK[4*i+2*j+k-7,4]=Teta5[i-1][j-1][k-1]*180/pi
                    t5=Teta5[i-1][j-1][k-1]

                    #Teta6
                    Teta6[i-1][j-1][k-1]=(a-b)/2 - (k-1)*pi
                    InK[4*i+2*j+k-7,5]=Teta6[i-1][j-1][k-1]*180/pi
                    t6=Teta6[i-1][j-1][k-1]

        InK = np.mod((InK) + 180 ,360) - 180
        for p in range(8):
            for t in range(6):
                DInK[p][t] =np.abs( InK[p][t] - CurPos[t])
        DInK = np.abs(np.mod((DInK) + 180 ,360) - 180)
        DInK[:][0:3] = DInK[:][0:3] *3

        if H==1:
            InK = InK [0:4, :]
            DInK = DInK [0:4, :]
            S = np.sum(DInK, axis=1)
            i = np.argmin(S)
            Q = InK[i, :]
        else:
            S = np.sum(DInK, axis=1)
            i = np.argmin(S)
            Q = InK[i, :]

        a = Q[5] - CurPos[5]
        if np.abs(a) >= 359:
            a0 = a
            if a > 0:
                while abs(a0) > 1:
                    Q[5] -= 360
                    a0 = Q[5] - CurPos[5]
            else:

                while abs(a0) > 1:
                    Q[5] += 360
                    a0 = Q[5] - CurPos[5]

        # Modificatin of Wrist Singularity
        if abs(CurPos[4]) < 0.01812:
            Q46 = Q[3]+Q[5]
            Q[3] = CurPos[3]
            Q[5] = Q46 - Q[3]
            print('Wrist')

        # Modificatin of Reach(Elbow) Singularity
        if np.sqrt(Qg6**2+ Qg7**2+ (Qg8-389.5)**2) >= ((600+ np.sqrt(200**2+685.5**2)) -0.5):
            Q[1] = CurPos [1]
            print('Reach')

        return Q
        

    def angle2quat(self, yaw, pitch, roll):
        # Convert angle to quateriun
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        q = [qw, qx, qy, qz]
        return q
    
    def DQmultiply(self, DQ1, DQ2):
        # Correct
        # Verified byexample
        # v1 x v2 is correct
        s1 = DQ1[0]; v1 = np.array(DQ1[1:4]); p1 = np.array(DQ1[5:8]); 
        s2 = DQ2[0]; v2 = np.array(DQ2[1:4]); p2 = np.array(DQ2[5:8]); 
        DQ = np.zeros(8)
        DQ[0:4] = np.append([s1*s2 - sum(self.valmultiply(v1, v2))], [s1*v2 + s2*v1 + np.cross(v1, v2)])
        DQ[5:8] = p1 + p2 + 2*s1 * np.cross(v1,p2) + 2*np.cross(v1, np.cross(v1,p2))

        return DQ
    

    def DQinv(self, q0):
        # Correct!
        # Verified by example
        s = q0[0]
        v = np.array(q0[1:4])
        p = np.array(q0[5:8])
        Q = np.zeros(8)
        Q[0:4] = np.append([s] ,[(-1)*v])
        Q[5:8] = -p+(-2*s*(np.cross(v,(-1)*p))+2*np.cross(v,np.cross(v,(-1)*p)))
        return Q.tolist()
    

    def valmultiply(self, L1,L2):
        ans = []
        for i in range(0, len(L1)):
            ans.append(L1[i] * L2[i])
        return ans

 
# def main(args=None):
#     x=[]
#     y=[]
#     z=[]
#     joint_in=[]
#     z=np.linspace(1202,1000,1000)
#     x=np.linspace(685.5,685.5,1000)
#     y=np.linspace(0,0,1000)
#     invkin = inv_kin()
#     t1=0
#     t2=0
#     t3=0
#     t4=0
#     t5=0
#     t6=0
    
#     for i in range(1000):
#         Q = invkin.inv_k(x[i],y[i],z[i],0,0,0,t1,t2,t3,t4,t5,t6)
#         t1= Q[0]
#         t2=Q[1]
#         t3=Q[2]
#         t4=Q[3]
#         t5=Q[4]
#         t6=Q[5]
#         print(Q)
#         joint_in.append(Q * np.pi/180)
#     print(joint_in)
    
#     return joint_in
    
# if __name__ == '__main__':
#     main()
