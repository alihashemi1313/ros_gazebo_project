import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Obstcle():
    def __init__(self) -> None:
        pass


    def obstcle_pointClud(self, Obslength, centerPoint, pointnum):
        '''
        Function Defintion:

        This function recive the obstcle parameters and a point
        and returns the point cloud of each face of the object
        '''
        
        # Calculate the lower and uper bound for obstcle
        obs_xl= centerPoint[0] - Obslength[0]/2;    obs_xu= centerPoint[0] + Obslength[0]/2
        obs_yl= centerPoint[1] - Obslength[1]/2;    obs_yu= centerPoint[1] + Obslength[1]/2
        obs_zl= centerPoint[2] - Obslength[2]/2;    obs_zu= centerPoint[2] + Obslength[2]/2


        # Up Face
        xup = np.random.uniform(obs_xl, obs_xu, pointnum)
        yup = np.random.uniform(obs_yl, obs_yu, pointnum)
        zup = obs_zu * np.ones(len(xup))

        point_cloud_up = np.vstack((xup, yup, zup)).T

        # Down Face
        xdwn = np.random.uniform(obs_xl, obs_xu, pointnum)
        ydwn = np.random.uniform(obs_yl, obs_yu, pointnum)
        zdwn = obs_zl * np.ones(len(xdwn))

        point_cloud_dwn = np.vstack((xdwn, ydwn, zdwn)).T

        # Left Face
        xleft = np.random.uniform(obs_xl, obs_xu, pointnum)
        zleft = np.random.uniform(obs_zl, obs_zu, pointnum)
        yleft = obs_yl * np.ones(len(xleft))

        point_cloud_left = np.vstack((xleft, yleft, zleft)).T

        # Rigth Face
        xrigth = np.random.uniform(obs_xl, obs_xu, pointnum)
        zrigth = np.random.uniform(obs_zl, obs_zu, pointnum)
        yrigth = obs_yu * np.ones(len(xrigth))

        point_cloud_rigth = np.vstack((xrigth, yrigth, zrigth)).T

        # Front Face
        yfront = np.random.uniform(obs_yl, obs_yu, pointnum)
        zfront = np.random.uniform(obs_zl, obs_zu, pointnum)
        xfront = obs_xu * np.ones(len(yfront))

        point_cloud_front = np.vstack((xfront, yfront, zfront)).T

        # Back Face
        yback = np.random.uniform(obs_yl, obs_yu, pointnum)
        zback = np.random.uniform(obs_zl, obs_zu, pointnum)
        xback = obs_xl * np.ones(len(yback))

        point_cloud_back = np.vstack((xback, yback, zback)).T
        

        return point_cloud_up, point_cloud_dwn, point_cloud_left, point_cloud_rigth, point_cloud_front, point_cloud_back
    
    def robot_shape(self, Theta):
        
        pi = math.pi
        # Robot parameters
        L1 = 389.5; L2 = 0; L3 = 600 + 200; L4 = 0; L5 = 685.5; # in [mm]

        t1=Theta[0] *pi/180
        t2=Theta[1] *pi/180
        t3=Theta[2] *pi/180
        t4=Theta[3] *pi/180
        t5=Theta[4] *pi/180
        t6=Theta[5] *pi/180

        # Direct Kinematic
        Q1 = np.append(np.append(math.cos(t1/2), math.sin(t1/2)*np.array([0, 0, 1])), [0 ,0 ,0 ,L1])
        Q2 = np.append(np.append(math.cos(t2/2), math.sin(t2/2)*np.array([0, 1, 0])), [0 ,L2 ,0 ,0])
        Q3 = np.append(np.append(math.cos(t3/2), math.sin(t3/2)*np.array([0, 1, 0])), [0 ,0 ,0 ,L3])
        Q4 = np.append(np.append(math.cos(t4/2), math.sin(t4/2)*np.array([1, 0, 0])), [0 ,L5 ,0 ,L4])
        Q5 = np.append(np.append(math.cos(t5/2), math.sin(t5/2)*np.array([0, 1, 0])), [0 ,0 ,0 ,0])
        Q6 = np.append(np.append(math.cos(t6/2), math.sin(t6/2)*np.array([1, 0, 0])), [0 ,135 ,0 ,0])
        QT = np.append(np.append(math.cos(0), math.sin(0)*np.array([1, 0, 0])), [0 ,0 ,0 ,0])

        M1 = Q1
        M2 = self.DQmultiply(M1, Q2)
        JointPoint1 = M2[5:9]

        M3 = self.DQmultiply(M2, Q3)
        JointPoint2 = M3[5:9]

        M4 = self.DQmultiply(M3, Q4)
        JointPoint3 = M4[5:8]

        M5 = self.DQmultiply(M4, Q5)
        M6 = self.DQmultiply(M5, Q6)

        JointPoint4 = M6[5:9]

        linkPoints0 = {'x' : np.linspace(0, JointPoint1[0], 10),
                    'y' : np.linspace(0, JointPoint1[1], 10),
                    'z' : np.linspace(0, JointPoint1[2], 10) }
        
        
        linkPoints1 = {'x' : np.linspace(JointPoint1[0], JointPoint2[0], 10),
                    'y' : np.linspace(JointPoint1[1], JointPoint2[1], 10),
                    'z' : np.linspace(JointPoint1[2], JointPoint2[2], 10) }
        
        linkPoints2 = {'x' : np.linspace(JointPoint2[0], JointPoint3[0], 10),
                    'y' : np.linspace(JointPoint2[1], JointPoint3[1], 10),
                    'z' : np.linspace(JointPoint2[2], JointPoint3[2], 10) }
        
        linkPoints3 = {'x' : np.linspace(JointPoint3[0], JointPoint4[0], 10),
                    'y' : np.linspace(JointPoint3[1], JointPoint4[1], 10),
                    'z' : np.linspace(JointPoint3[2], JointPoint4[2], 10) }
        
        x = np.append(np.append(np.append(linkPoints0['x'], linkPoints1['x']), linkPoints2['x']), linkPoints3['x'])
        y = np.append(np.append(np.append(linkPoints0['y'], linkPoints1['y']), linkPoints2['y']), linkPoints3['y'])
        z = np.append(np.append(np.append(linkPoints0['z'], linkPoints1['z']), linkPoints2['z']), linkPoints3['z'])

        point = np.array([x,y,z])
        
        
        # fig = plt.figure()
        # ax = fig.add_subplot(111, projection='3d')

        # ax.scatter(linkPoints0['x'],    linkPoints0['y'],    linkPoints0['z'],    c='k', marker='o', s=3)
        # ax.scatter(linkPoints1['x'],    linkPoints1['y'],    linkPoints1['z'],    c='r', marker='o', s=3)
        # ax.scatter(linkPoints2['x'],    linkPoints2['y'],    linkPoints2['z'],    c='g', marker='o', s=3)
        # ax.scatter(linkPoints3['x'],    linkPoints3['y'],    linkPoints3['z'],    c='b', marker='o', s=3)
        
        # # ax.plot3D(Pose1[0], Pose1[1], Pose1[2], linewidth=4, c = 'r')
        
        # ax.set_xlabel('X')
        # ax.set_ylabel('Y')
        # ax.set_zlabel('Z')
        # ax.set_title('Cubic Point Cloud Obstcle')
        # plt.show()

        return point
    


    def avoid_obstcle(self, Obslength, centerPoint, pathPoint):

        '''
        Function Defintion:

        This function recive the obstcle parameters and a point
        and returns the point cloud of the object
        '''

        pc_up, pc_dwn, pc_left, pc_rigth, pc_front, pc_back = self.obstcle_pointClud(Obslength, centerPoint, 30)

        # All Point Cludes 
        pointCloud = np.vstack((pc_up, pc_dwn, pc_left, pc_rigth, pc_front, pc_back)).T
        s = pointCloud.shape

        dis = np.zeros(s[1])
        for j in range(s[1]):

            dis[j] = math.sqrt((pointCloud[0][j] - pathPoint[0])**2 + (pointCloud[1][j] - pathPoint[1])**2 + (pointCloud[2][j] - pathPoint[2])**2)

        index = np.argmin(dis)
        clossestPoint = [pointCloud[0][index], pointCloud[1][index], pointCloud[2][index]]
        mindis = min(dis)

        return mindis, clossestPoint
    


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

    def valmultiply(self, L1,L2):
        ans = []
        for i in range(0, len(L1)):
            ans.append(L1[i] * L2[i])
        return ans









def main(args=None):

    # Create an object
    creatOBS = Obstcle()

    mindis, clossest = creatOBS.avoid_obstcle([5,5,10], [10,10,10], [2,2,2])
    print('the minimum distance is: ', mindis)
    print('the clossest point is: ', clossest)

    Th = [10, 20, 30, 40, 50, 60]

    p = creatOBS.robot_shape(Th)
    print(p)




if __name__ == '__main__':
    main()