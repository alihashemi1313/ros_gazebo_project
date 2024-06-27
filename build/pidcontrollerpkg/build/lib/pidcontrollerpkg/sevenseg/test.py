import csv
import sys, os
from seven_segment_trajectory import SevenSegmentTrajectory
import pandas as pd
from decimal import *
import numpy as np
getcontext().prec=16
def build_trajectory(theta,joint_in):
        ftraj=[]
        q1=[]
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
            for i in range(0,int(0.5*1000+1)):
                if i<len(traj.q):
                    q.append(traj.q[i])
                else:
                    q.append(Decimal(joint_in[j]))
            ftraj.append(q)
        ftraj=np.asarray(ftraj,dtype=float)
        print(ftraj[0][500])
        print("ok")
        print("Trajectory Build")
        return ftraj
    
joint_in=[0.01,0.02,0.03,0.04,0.05,0.06]
theta=[0,0,0,0,0,0]

ftraj=build_trajectory(theta,joint_in)

# print(ftraj)