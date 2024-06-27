import sys
import tempfile
from sympy import symbols, cos, sin, pi, simplify, pprint, tan, expand_trig, sqrt, trigsimp, atan2,acos
from sympy.matrices import Matrix
import numpy as np
import math

import pyswarms as ps
from pyswarms.utils.functions import single_obj as fx
from pyswarms.single.global_best import GlobalBestPSO

def pose(theta, alpha, a, d):
  # returns the pose T of one joint frame i with respect to the previous joint frame (i - 1)
  # given the parameters:
  # theta: theta[i]
  # alpha: alpha[i-1]
  # a: a[i-1]
  # d: d[i]

  r11, r12 = cos(theta), -sin(theta)
  r23, r33 = -sin(alpha), cos(alpha)
  r21 = sin(theta) * cos(alpha)
  r22 = cos(theta) * cos(alpha)
  r31 = sin(theta) * sin(alpha)
  r32 = cos(theta) * sin(alpha)
  y = -d * sin(alpha)
  z = d * cos(alpha)
    
  T = Matrix([
    [r11, r12, 0.0, a],
    [r21, r22, r23, y],
    [r31, r32, r33, z],
    [0.0, 0.0, 0.0, 1]
  ])
  
  T = simplify(T)

  return T

# under construction
def forward_kin(q1,q2,q3,q4,q5,q6):
  X = []
  Y = []
  Z = []
  d90 = pi/2
  T01 = pose(q1, 0, 0, 0.326)
  T0g = T01
  px,py,pz = T0g[0,3], T0g[1,3], T0g[2,3]
  X.append(px)
  Y.append(py)
  Z.append(pz)
  T12 = pose(q2 - d90 , -d90, 0, 0)
  T0g = T0g* T12
  px,py,pz = T0g[0,3], T0g[1,3], T0g[2,3]
  X.append(px)
  Y.append(py)
  Z.append(pz)
  T23 = pose(q3, 0, 0.6, 0)
  T0g = T0g* T23
  px,py,pz = T0g[0,3], T0g[1,3], T0g[2,3]
  X.append(px)
  Y.append(py)
  Z.append(pz)
  T34 = pose(q4, -d90, 0.2, 0.6855)
  T0g = T0g* T34
  px,py,pz = T0g[0,3], T0g[1,3], T0g[2,3]
  X.append(px)
  Y.append(py)
  Z.append(pz)
  T45 = pose(-q5, -d90, 0, 0)
  T0g = T0g* T45
  px,py,pz = T0g[0,3], T0g[1,3], T0g[2,3]
  X.append(px)
  Y.append(py)
  Z.append(pz)
  T56 = pose(q6, d90, 0, 0.079)
  T0g = T0g* T56
  px,py,pz = T0g[0,3], T0g[1,3], T0g[2,3]
  X.append(px)
  Y.append(py)
  Z.append(pz)
  #end effector
  T6g = pose(0, 0, 0, 0.056)
  #final position and rotation
  T0g = T0g* T6g
  px,py,pz = T0g[0,3], T0g[1,3], T0g[2,3]
  X.append(px)
  Y.append(py)
  Z.append(pz)
  
  #fig = plt.figure()
  #ax = fig.add_subplot(111,projection = '3d')
  #ax.set_xlabel('x axis')
  #ax.set_ylabel('y axis')
  #ax.set_zlabel('z axis')
  
  X = np.reshape(X,(1,7))
  Y = np.reshape(Y,(1,7))
  Z = np.reshape(Z,(1,7))
  
  X , Y , Z = X[0][6],Y[0][6],Z[0][6]
  roll = atan2(T0g[2,0],T0g[2,1])
  pitch = acos(T0g[2,2])
  yaw = -atan2(T0g[0,2],T0g[1,2])

  return X,Y,Z,roll,pitch,yaw

def fitness_1(x):
    L = forward_kin(x[:,0],x[:,1],x[:,2],x[:,3],x[:,4],x[:,5])
    L_d = [0.5,0.5,0.5,pi/2,pi/2,-pi/2]
    f = ((L_d[0]-L[0])+(L_d[1]-L[1])+(L_d[2]-L[2])+(L_d[3]-L[3])+(L_d[4]-L[4])+(L_d[5]-L[5]))**2
    return f

max_bound = pi * np.ones(6)
min_bound = -pi * np.ones(6)
bounds = (min_bound, max_bound)

options = {'c1': 2, 'c2': 2, 'w': 0.9}
optimizer = GlobalBestPSO(n_particles=10, dimensions=6, options=options)
cost, pos = optimizer.optimize(fitness_1,iters=50)