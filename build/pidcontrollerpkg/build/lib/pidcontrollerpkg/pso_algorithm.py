import numpy as np
import time

# ----------------------------------------#

class Optimizer():

    def __init__(self):
        pass

    def PSO(self,xL,xU,fitness,InitialSol,CtrlParam):
        

        # Control parameters
        w     = CtrlParam['w']                   # Intertial weight
        c1    = CtrlParam['c1']                  # Weight of searching based on the optima found by a particle
        c2    = CtrlParam['c2']                  # Weight of searching based on the optima found by the swarm
        v_fct = CtrlParam['v_fct']               # Velocity adjust factor


        Np       = CtrlParam['Np']               # population size
        D        = CtrlParam['D']                # dimension 
        max_iter = CtrlParam['max_iter']         # maximum number of iterations 


        # Defining and intializing variables
        
        pbest_val = np.zeros(Np)            # Personal best fintess value. One pbest value per particle.
        gbest_val = np.zeros(max_iter)      # Global best fintess value. One gbest value per iteration (stored).

        pbest = np.zeros((D,Np))            # pbest solution
        gbest = np.zeros(D)                 # gbest solution

        gbest_store = np.zeros((D,max_iter))   # storing gbest solution at each iteration

        pbest_val_avg_store = np.zeros(max_iter)
        fitness_avg_store = np.zeros(max_iter)

        x = InitialSol['x']            # Initial position of the particles
        v = InitialSol['v']            # Initial velocity of the particles

        if len(x) == 19:
            tmin = 2; tmax = 10
            xL = np.append(xL, tmin)
            xU = np.append(xU, tmin)

        
        # Setting the initial position of the particles over the given bounds [xL,xU]
        for m in range(D):    
            x[m,:] = xL[m] + (xU[m]-xL[m])*x[m,:]
            



        t_start = time.time()

        # Loop over the generations
        for iter in range(0,max_iter):
            
            if iter > 0:                             # Do not update postion for 0th iteration
                r1 = np.random.rand(D,Np)            # random numbers [0,1], matrix D x Np
                r2 = np.random.rand(D,Np)            # random numbers [0,1], matrix D x Np   
                v_global = np.multiply(((x.transpose()-gbest).transpose()),r2)*c2*(-1.0)    # velocity towards global optima
                v_local = np.multiply((pbest- x),r1)*c1           # velocity towards local optima (pbest)
            
                v = w*v + (v_local + v_global)       # velocity update
                x = x + v*v_fct                      # position update
            
            
            
            
            fit = fitness(x)         # fitness function call (once per iteration). Vector Np
    
            
            if iter == 0:
                pbest_val = np.copy(fit)             # initial personal best = initial fitness values. Vector of size Np
                pbest = np.copy(x)                   # initial pbest solution = initial position. Matrix of size D x Np
            else:
                # pbest and pbest_val update
                ind = np.argwhere(fit > pbest_val)   # indices where current fitness value set is greater than pbset
                pbest_val[ind] = np.copy(fit[ind])   # update pbset_val at those particle indices where fit > pbest_val
                pbest[:,ind] = np.copy(x[:,ind])     # update pbest for those particle indices where fit > pbest_val
            
            # gbest and gbest_val update
            ind2 = np.argmax(pbest_val)                       # index where the fitness is maximum
            gbest_val[iter] = np.copy(pbest_val[ind2])        # store gbest value at each iteration
            gbest = np.copy(pbest[:,ind2])                    # global best solution, gbest
            
            gbest_store[:,iter] = np.copy(gbest)              # store gbest solution
            
            pbest_val_avg_store[iter] = np.mean(pbest_val)
            fitness_avg_store[iter] = np.mean(fit)
            print("Iter. =", iter, ". gbest_val = ", (-1)*gbest_val[iter])  # print iteration no. and best solution at each iteration
            
            

        t_elapsed = time.time() - t_start
        print("\nElapsed time = %.4f s" % t_elapsed)


        NextInitialSol = {'x': x, 'v': v}

        return NextInitialSol, gbest


