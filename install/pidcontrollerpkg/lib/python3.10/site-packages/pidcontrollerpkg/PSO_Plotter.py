import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import numpy as np

class Plotter():
    def __init__(self) -> None:
        pass

    ###############################################
    ##             Plotting results              ##
    ###############################################
    
    def plotter(T, P, ThD , ThA):
        
        fig2 , axs2 = plt.subplots(3)
        fig2.suptitle(" FUM6R Cortisian Trajectory ")

        axs2[0].plot(T,P[0])
        axs2[0].set_title('[t, x]')

        axs2[1].plot(T,P[2])
        axs2[1].set_title('[t, z]')
            
        axs2[2].plot(P[0],P[2])
        axs2[2].set_title('[x, z]')

        plt.show()

        T = np.append([0.0], T)

        fig , axs = plt.subplots(3,2)
        fig.suptitle(" FUM6R Joint Trajectory ")

        axs[0, 0].plot(T,ThD[0], T,ThA[0])
        axs[0, 0].set_title('Theta1')

        axs[0, 1].plot(T,ThD[1], T,ThA[1])
        axs[0, 1].set_title('Theta2')

        axs[1, 0].plot(T,ThD[2], T,ThA[2])
        axs[1, 0].set_title('Theta3')

        axs[1, 1].plot(T,ThD[3], T,ThA[3])
        axs[1, 1].set_title('Theta4')

        axs[2, 0].plot(T,ThD[4], T,ThA[4])
        axs[2, 0].set_title('Theta5')

        axs[2, 1].plot(T,ThD[5], T,ThA[5])
        axs[2, 1].set_title('Theta6')

        plt.show()

    def plotter2(T, P):
        
        fig2 , axs2 = plt.subplots(3)
        fig2.suptitle(" FUM6R Cortisian Trajectory ")

        axs2[0].plot(T,P[0])
        axs2[0].set_title('[t, x]')

        axs2[1].plot(T,P[2])
        axs2[1].set_title('[t, z]')
            
        axs2[2].plot(P[0],P[2])
        axs2[2].set_title('[x, z]')

        plt.show()

        ax = plt.axes(projection='3d')
        ax.plot3D(P[0], P[1], P[2], 'gray')
