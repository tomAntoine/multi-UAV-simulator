# -*- coding: utf-8 -*-
"""
author: John Bass
email: john.bobzwik@gmail.com
license: MIT
Please feel free to use and modify this, but keep the above information. Thanks!

adaptation
author: Tom Antoine and Alex Martinez
"""


import numpy as np
import matplotlib.pyplot as plt
import time
import cProfile
from trajectory import Trajectory
from ctrl import Control
from quadFiles.quad import Quadcopter
from utils.windModel import Wind
import utils
import config
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib.legend import Legend
import random

def quad_sim(t, Ts, quads, wind, i):

    pos_quads = []
    for quad in quads:
        pos_quads.append([quad.quad_id,quad.pos])

    for quad in quads:

        quad.pos_quads = pos_quads

        if  quad.neutralize :

            quads[quad.id_targ].mode = "fall"

            quad.neutralize = False

            for quad_2 in quads:
                if quad_2.id_targ == quad.id_targ and quad_2 != quad:
                    quad_2.mode = "home"

        quad.update(t, Ts, wind, i, quad)


Ti = 0
Ts = 0.005
Tf = 10
ifsave = 0


# Simulation Setup
# ---------------------------

# Choose trajectory settings
# ---------------------------
ctrlOptions = ["xyz_pos", "xy_vel_z_pos", "xyz_vel"]
trajSelect = np.zeros(3)

# Select Control Type             (0: xyz_pos,                  1: xy_vel_z_pos,            2: xyz_vel)
ctrlType = ctrlOptions[0]
# Select Position Trajectory Type (0: hover,                    1: pos_waypoint_timed,      2: pos_waypoint_interp,
#                                  3: minimum velocity          4: minimum accel,           5: minimum jerk,           6: minimum snap
#                                  7: minimum accel_stop        8: minimum jerk_stop        9: minimum snap_stop
#                                 10: minimum jerk_full_stop   11: minimum snap_full_stop
#                                 12: pos_waypoint_arrived
trajSelect[0] = 3
# Select Yaw Trajectory Type      (0: none                      1: yaw_waypoint_timed,      2: yaw_waypoint_interp     3: follow          4: zero)
trajSelect[1] = 0
# Select if waypoint time is used, or if average speed is used to calculate waypoint time   (0: waypoint time,   1: average speed)
trajSelect[2] = 1
print("Control type: {}".format(ctrlType))

# Initialize Quadcopter, Controller, Wind, Result Matrixes

    # Initialize Result Matrixes
# ---------------------------
numTimeStep = int(Tf/Ts+1)

# ---------------------------


def full_scenario():
    pos_obs = np.array([[1, 5, -2], [8, 2, -8], [5, 8, -9], [0, 0, -2], [3, 3, -1],[3, 9, -17],[5, 7, -18],[0, 0, -10],[5, 10, -16],[10,10,-12],[13,13,-13]])
    quad0 = Quadcopter(Ti, Tf, ctrlType, trajSelect, numTimeStep, Ts, quad_id = 0, mode='ennemy', id_targ = -1, color = 'blue', pos_ini = [0,0,0], pos_goal= [15,15,-15], pos_obs = pos_obs)
    quad1 = Quadcopter(Ti, Ts*90, ctrlType, trajSelect, numTimeStep, Ts, quad_id = 1, mode='guided', id_targ = -1, color = 'green', pos_ini = [0,3,0], pos_goal = [15,10,-15], pos_obs = pos_obs)
    quad2 = Quadcopter(Ti, Ts*100, ctrlType, trajSelect, numTimeStep, Ts, quad_id = 2,  mode='track', id_targ = 0, color = 'pink', pos_ini = [3,0,0], pos_goal = [15,20,-15], pos_obs = pos_obs)
    quads = [quad0, quad1, quad2]
    return pos_obs,quads

def multi_waypoint_scenario():
    pos_obs = np.array([[50,0,0]])
    quad0 = Quadcopter(Ti, Tf, ctrlType, trajSelect, numTimeStep, Ts, quad_id = 0, mode='ennemy', id_targ = -1, color = 'blue', pos_ini = [0,0,0], pos_goal= [0,-17,-10], pos_obs = pos_obs)
    quad1 = Quadcopter(Ti, Tf, ctrlType, trajSelect, numTimeStep, Ts, quad_id = 1, mode='ennemy', id_targ = -1, color = 'green', pos_ini = [20,0,0], pos_goal = [-20,-15,-10], pos_obs = pos_obs)
    quad2 = Quadcopter(Ti, Tf, ctrlType, trajSelect, numTimeStep, Ts, quad_id = 2, mode='ennemy', id_targ = -1, color = 'red', pos_ini = [-20,-10,0], pos_goal = [-10,0,-20], pos_obs = pos_obs)
    quads = [quad0, quad1, quad2]
    return pos_obs,quads

def static_OA_scenario():
    pos_obs = []
    for i in range(30):
        pos_obs.append(random.sample(range(-10, 0), 3))
    pos_obs = np.array(pos_obs)
    quad0 = Quadcopter(Ti, Tf, ctrlType, trajSelect, numTimeStep, Ts, quad_id = 0, mode='ennemy', id_targ = -1, color = 'blue', pos_ini = [0,0,0], pos_goal= [-10,-10,-10], pos_obs = pos_obs)
    quads = [quad0]
    return pos_obs,quads

def dynamic_CA_scenario():
    #Tf =8s
    pos_obs = np.array([[50,0,0]])
    quad0 = Quadcopter(Ti, Ts*100, ctrlType, trajSelect, numTimeStep, Ts, quad_id = 0, mode='guided', id_targ = -1, color = 'blue',  pos_ini = [0,10,-5],pos_goal = [30,10,-5], pos_obs = pos_obs)
    x, z = random.randint(3,17),-1*random.randint(1,8)
    quad1 = Quadcopter(Ti, Tf, ctrlType, trajSelect, numTimeStep, Ts, quad_id = 1, mode='ennemy', id_targ = -1, color = 'green', pos_ini = [x,0,z], pos_goal = [x,20,z], pos_obs = pos_obs)
    x, z = random.randint(3,17),-1*random.randint(1,8)
    quad2 = Quadcopter(Ti, Tf, ctrlType, trajSelect, numTimeStep, Ts, quad_id = 2, mode='ennemy', id_targ = -1, color = 'green', pos_ini = [x,0,z], pos_goal = [x,20,z], pos_obs = pos_obs)
    x, z = random.randint(3,17),-1*random.randint(1,8)
    quad3 = Quadcopter(Ti, Tf, ctrlType, trajSelect, numTimeStep, Ts, quad_id = 3, mode='ennemy', id_targ = -1, color = 'green', pos_ini = [x,0,z], pos_goal = [x,20,z], pos_obs = pos_obs)
    quads = [quad0, quad1,quad2,quad3]
    return pos_obs,quads

def simple_tracking_scenario():
    pos_obs = np.array([[-10,-10,0]])
    quad0 = Quadcopter(Ti, Tf, ctrlType, trajSelect, numTimeStep, Ts, quad_id = 0, mode='ennemy', id_targ = -1, color = 'blue',  pos_ini = [0,0,0],  pos_goal = [15,15,-15],  pos_obs = pos_obs)
    quad1 = Quadcopter(Ti, Ts*90, ctrlType, trajSelect, numTimeStep, Ts, quad_id = 1, mode='track', id_targ = 0, color = 'green', pos_ini = [5,5,0], pos_goal = [2,2,-10],  pos_obs = pos_obs)
    quads = [quad0, quad1]
    return pos_obs,quads

def multi_tracking_scenario():
    pos_obs = np.array([[-10,-10,0]])
    quad0 = Quadcopter(Ti, Tf, ctrlType, trajSelect, numTimeStep, Ts, quad_id = 0, mode='ennemy', id_targ = -1, color = 'blue',  pos_ini = [0,0,0],  pos_goal = [15,15,-15],  pos_obs = pos_obs)
    quad1 = Quadcopter(Ti, Ts*100, ctrlType, trajSelect, numTimeStep, Ts, quad_id = 1, mode='track', id_targ = 0, color = 'green', pos_ini = [4,0,0], pos_goal = [4,4,-10],  pos_obs = pos_obs)
    quad2 = Quadcopter(Ti, Ts*100, ctrlType, trajSelect, numTimeStep, Ts, quad_id = 2, mode='track', id_targ = 0, color = 'green', pos_ini = [4,4,0], pos_goal = [4,4,-10],  pos_obs = pos_obs)
    quad3 = Quadcopter(Ti, Ts*100, ctrlType, trajSelect, numTimeStep, Ts, quad_id = 3, mode='track', id_targ = 0, color = 'green', pos_ini = [4,-4,0], pos_goal = [4,4,-10],  pos_obs = pos_obs)
    quads = [quad0, quad1, quad2, quad3]
    return pos_obs,quads

def tracking_loop_scenario(x):
    pos_obs = np.array([[x/2,x/2,-10]])
    quad0 = Quadcopter(Ti, Ts*99, ctrlType, trajSelect, numTimeStep, Ts, quad_id = 0, mode='track', id_targ = 1, color = 'blue',  pos_ini = [0,0,-10],  pos_goal = [0,x,-10],  pos_obs = pos_obs)
    quad1 = Quadcopter(Ti, Ts*100, ctrlType, trajSelect, numTimeStep, Ts, quad_id = 1, mode='track', id_targ = 2, color = 'green', pos_ini = [x,0,-10], pos_goal = [0,0,-10],  pos_obs = pos_obs)
    quad2 = Quadcopter(Ti, Ts*101, ctrlType, trajSelect, numTimeStep, Ts, quad_id = 2, mode='track', id_targ = 3, color = 'orange', pos_ini = [x,x,-10],pos_goal = [x,0,-10], pos_obs = pos_obs)
    quad3 = Quadcopter(Ti, Ts*102, ctrlType, trajSelect, numTimeStep, Ts, quad_id = 3, mode='track', id_targ = 0, color = 'pink', pos_ini = [0,x,-10], pos_goal = [x,x,-10],pos_obs = pos_obs)
    quads = [quad0, quad1,quad2,quad3]
    return pos_obs,quads

def tracking_and_kill_scenario():
    pos_obs = np.array([[-10,-10,0]])
    quad0 = Quadcopter(Ti, Tf, ctrlType, trajSelect, numTimeStep, Ts, quad_id = 0, mode='ennemy', id_targ = -1, color = 'blue',  pos_ini = [0,0,-5],  pos_goal = [20,15,-20],  pos_obs = pos_obs)
    quad1 = Quadcopter(Ti, Ts*100, ctrlType, trajSelect, numTimeStep, Ts, quad_id = 1, mode='track', id_targ = 0, color = 'green', pos_ini = [5,0,0], pos_goal = [4,4,-10],  pos_obs = pos_obs)
    quads = [quad0, quad1]
    return pos_obs,quads

def dist(a,b):
    return np.sqrt((a[0]-b[0])**2+(a[1]-b[1])**2+(a[2]-b[2])**2)

def getmindist(pos_all,pos_obs):
    dist_i = []
    for pos in  pos_all :
        temp = []
        for obs in pos_obs:
            temp.append(dist(pos,obs))
        dist_i.append(min(temp))
    return(min(dist_i))


def getmindistother(pos_all):
    pos_guided = pos_all[0]
    dist_i = []
    for t in  range(len(pos_guided)) :
        temp = []
        for i in range(len(pos_all)-1):
            temp.append(dist(pos_guided[t],pos_all[i+1][t]))
        dist_i.append(min(temp))
    return(min(dist_i))


def getgoaldist(pos_all,pos_goal):
    temp = []
    for d in pos_all:
        temp.append(dist(d,pos_goal))
    return(min(temp))


def main():

    start_time = time.time()
    pos_obs,quads = dynamic_CA_scenario()

    wind = Wind('None', 2.0, 90, -15)

    for quad in quads:
        quad.init(Ts,quad)

    # Run Simulation
    # ---------------------------
    t = Ti
    i = 1
    t_int = 0
    while round(t,3) < Tf:
        quad_sim(t, Ts, quads, wind, i)
        t += Ts
        i += 1

    end_time = time.time()
    print(end_time-start_time)
    pos_all = []
    for quad in quads:
        pos_all.append(quad.pos_all)
    min_dist_other = getmindistother(pos_all)
    #goaldist = getgoaldist(pos_all,quads[0].pos_goal_ini)
    #time_tot = end_time - start_time
    return min_dist_other


def plot_hist(data,title):
    fig, ax = plt.subplots()
    maxi = max(data)
    mini = min(data)
    bins = np.arange(mini,maxi,10)
    ax.hist(data)
    ax.set_ylabel('Number of Simulation')
    ax.set_title(title)


if __name__ == "__main__":

    min_dist_obs = []
    time_tot = []
    goaldist = []
    min_dist_other = []

    for i in range(100):
        print(i)
        min_dist_other_i = main()
        min_dist_other.append(min_dist_other_i)

    #plot_hist(min_dist_obs,'Minimum distance to obstacle')
    #plot_hist(time_tot,'Computational Time')
    #plot_hist(goaldist,'Final distance to goal')
    plot_hist(min_dist_other,'Minimum Distance to Other Agents')

    """
    print("mean min dist")
    print(sum(min_dist_obs)/len(min_dist_obs))
    print("mean time")
    print(sum(time_tot)/len(time_tot))
    print("mean final goal dist")
    print(sum(goaldist)/len(goaldist))
    print("proportion of crash")
    print(sum(i < 0.3 for i in min_dist_obs)/len(min_dist_obs)*100)
    print("proportion of crash with margin")
    print(sum(i < 0.5 for i in min_dist_obs)/len(min_dist_obs)*100)
    """
    print(sum(min_dist_other)/len(min_dist_other))
    print(sum(i < 0.5 for i in min_dist_other)/len(min_dist_other)*100)

    plt.show()
