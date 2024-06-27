import sys
import tempfile
import ikpy
from ikpy.chain import Chain

import numpy as np
import math

urdf_file_path = "/home/aliha1313/fum6r_urdf.urdf"

robot = Chain.from_urdf_file(
        urdf_file_path,
        base_elements=[
            "6RL0", "6RJ1",
            "6RL1", "6RJ2", "6RL2", "6RJ3", "6RL3", "6RJ4", "6RL4",
            "6RJ5", "6RL5", "6RJ6", "6RL6"
        ],
        last_link_vector=[0, 0.056, 0],
        active_links_mask=[
            False, True, True, True, True, True, True, False
        ])

target_position = [0.5, 1, 1]
target_orientation = [0,0,0]

ik = robot.inverse_kinematics(target_position,target_orientation, orientation_mode="all")
invkin=[]
for i in range(8):
    ik[i] = ik[i] % 2*math.pi
    
for i in range(8):
    if ik[i]>math.pi:
        ik[i]=ik[i]-2*math.pi
        
invkin.append(ik[1])
invkin.append(ik[2])
invkin.append(ik[3])
invkin.append(ik[4])
invkin.append(ik[5])
invkin.append(ik[6])
print(invkin)