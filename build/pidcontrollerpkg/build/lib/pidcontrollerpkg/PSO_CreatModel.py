import numpy as np
from scipy import interpolate
import statistics
from pidcontrollerpkg.inverse_kinematics import inv_kin
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import csv
import pandas as pd

class CreatModel():

    def __init__(self) -> None:
        pass

    
    def Model(self, p, cond):

        xs= 685.5; ys= 0.0; zs= 1202; rs= 0.0; ps= 0.0; ws= 0.0                       #path planning start position
        xt= 786.9125; yt= 138.7539; zt= 556.7496; rt= -3.0858; pt= 1.0544; wt= 1.8190   #path planning target position 
        point = [[xs,ys,zs,rs,ps,ws], [xt,yt,zt,rt,pt,wt]]


        X = [xs, xs, p[0],  p[1],  p[2],  xt, xt]
        Y = [ys, ys, p[3],  p[4],  p[5],  yt, yt]
        Z = [zs, zs, p[6],  p[7],  p[8],  zt, zt]
        R = [rs, rs, p[9],  p[10], p[11], rt, rt]
        P = [ps, ps, p[12], p[13], p[14], pt, pt]
        W = [ws, ws, p[15], p[16], p[17], wt, wt]
        
        # print(len(p))
        if len(p) == 19:
            T = np.append(0.001, np.append(np.linspace(0.002, p[18]-0.001, num=5),p[18]))
        else:
            T = np.append(0.001, np.append(np.linspace(0.002, 1.0-0.001, num=5),1.0))
            
        #print

        
        tt = np.arange(min(T), max(T)+0.001, 0.001)

        # BSpline curver fitting 
        tck_x = interpolate.splrep(T, X, s=0, k=3)
        xx = interpolate.BSpline(*tck_x)(tt)        #BSpline (x,t)
        
        tck_y = interpolate.splrep(T, Y, s=0, k=3)
        yy = interpolate.BSpline(*tck_y)(tt)        #BSpline (y,t)

        tck_z = interpolate.splrep(T, Z, s=0, k=3)
        zz = interpolate.BSpline(*tck_z)(tt)        #BSpline (z,t)

        tck_r = interpolate.splrep(T, R, s=0, k=3)
        roll = interpolate.BSpline(*tck_r)(tt)      #BSpline (r,t)

        tck_p = interpolate.splrep(T, P, s=0, k=3)
        pitch = interpolate.BSpline(*tck_p)(tt)     #BSpline (p,t)
        
        tck_w = interpolate.splrep(T, W, s=0, k=3)
        yaw = interpolate.BSpline(*tck_w)(tt)       #BSpline (w,t)
        

        if cond:
            # Create Object of inverse kinematics class
            InvKin = inv_kin()

            Th1 =[0.0]; Th2 =[0.36048993]; Th3 =[-1.40912154]; Th4 =[0.0]; Th5 =[1.0486316]; Th6 =[0.0]
            cnt =0; pi = np.pi

            while cnt < len(xx):
                joint = InvKin.inv_k(xx[cnt], yy[cnt], zz[cnt], roll[cnt], pitch[cnt], yaw[cnt], 
                                     Th1[cnt], Th2[cnt], Th3[cnt], Th4[cnt], Th5[cnt], Th6[cnt])

                Th1.append(joint[0])
                Th2.append(joint[1])
                Th3.append(joint[2])
                Th4.append(joint[3])
                Th5.append(joint[4])
                Th6.append(joint[5])
                cnt +=1

            Theta = np.array([np.delete(Th1,0), np.delete(Th2,0), np.delete(Th3,0), np.delete(Th4,0), np.delete(Th5,0), np.delete(Th6,0)])
            
        else:

            Theta = np.array([])    
        

        Pose  = [xx, yy, zz, roll, pitch, yaw]
        time  = tt

        return time, Pose, Theta, point
    

def main(args=None):

    # Create an object
    Cmodel = CreatModel()
    p = [720.0,750.0,760.0, -200,300,50.0,400.0, 800.0,650.0, 0,0,0,1, 0,0.5,1.5, 0.5,0.9,10]
    time, Pose, theta, point = Cmodel.Model(p, True)
    
    fig, axs = plt.subplots(6)
    fig.suptitle('Vertically stacked subplots')
    print(theta[0])
    axs[0].plot(time, theta[0])
    axs[1].plot(time, theta[1])
    axs[2].plot(time, theta[2])
    axs[3].plot(time, theta[3])
    axs[4].plot(time, theta[4])
    axs[5].plot(time, theta[5])
    
    # fig, axs = plt.subplots(6)
    # fig.suptitle('Vertically stacked subplots')

    # axs[0].plot(time, Pose[0])
    # axs[1].plot(time, Pose[1])
    # axs[2].plot(time, Pose[2])
    # axs[3].plot(time, Pose[3])
    # axs[4].plot(time, Pose[4])
    # axs[5].plot(time, Pose[5])
    
    plt.show()
    
    df = pd.DataFrame(Pose)
    df.to_csv("pose.csv",header=False, index=False)
                
if __name__ == '__main__':
    main()
