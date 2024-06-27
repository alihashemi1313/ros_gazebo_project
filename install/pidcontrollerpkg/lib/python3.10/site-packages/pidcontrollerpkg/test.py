from inverse_kinematics import inv_kin
import numpy as np

x=[]
y=[]
z=[]
joint_in=[]
z=np.linspace(1200,1000,10000)
invkin = inv_kin()
t1=0
t2=0.0052569317*180/np.pi
t3=-0.0206210651*180/np.pi
t4=0
t5=0.0153641334*180/np.pi
t6=0

for i in range(10000):
    Q = invkin.inv_k(500,0,z[i],0,0,0,t1,t2,t3,t4,t5,t6)
    t1= Q[0]
    t2=Q[1]
    t3=Q[2]
    t4=Q[3]
    t5=Q[4]
    t6=Q[5]
    print(Q)

    joint_in.append(Q * np.pi/180)
