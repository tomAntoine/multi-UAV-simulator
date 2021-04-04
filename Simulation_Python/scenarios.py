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


"""
The variable “quad_id” is provided as an integer from 0 to the number of drones
in the simulation.

The “mode” is provided as a string and it can be split into three categories,
depending whether or not they are associated with the agent, the target or both.
The latter are simple actions such as “takeoff”, “home”, “land”, “fall” or
“charging”. Then, there are specific modes for agents like “guided” or “track”;
and targets like “enemy”. The change of mode can be pre-defined or provided
by the Mission planning and Task control subsystem. In the case of targets, the
transition is automated internally. They will be initialized in “enemy” mode and
changed into “neutralized” if the conditions are met to finally change into “fall”.
In the case of agents, the change of modes is performed externally after system
integration. However, due to the very intuitive transitions, some of them were
predefined in sequences for the subsystem validation and verification. For this
reason, “takeoff” and “land” mode were integrated at the beginning and end of
each mission. Similarly, after an agent in “track” mode neutralized its target, or
a “guided” one has reached its goal position, the mode was switched to “home”.
The “id_targ” is a specific integer input associated to the mode “track”. It
corresponds to the target identification number and is assigned as -1 by default
if any other mode is employed.

The “pos_goal” is a set of coordinates x, y and z in the global reference frame
that represent the goal position. It should be noted that although x and y are
not bounded, the z coordinate is restricted so that the drones cannot go through
the ground and by consistency with the guidance algorithms, it is defined as
negative. It should be noted that although this is an input from Mission planning
and Task control subsystem it will be updated for specific modes such as
“track”.

The “pos_obs” is a list of sets of coordinates x, y and z in the global reference
frame corresponding to the static obstacles and therefore should be kept
constant for all the drones in the simulation environment. This information is
predefined but will need to be provided by the Situation Awareness subsystem.

The “pos_ini” is a set of coordinates x, y and z in the global reference frame
that represent the initial position. It should be noted that as for the rest of
coordinates, the z coordinate is defined as negative.

The “color” is employed for the easy identification of the drones. It allows to
easily verify the correct functioning of the algorithms.

The “ctrlType” xyz_pos by default.

The “trajSelect” minimum velocity, no yaw control, average speedby default.

The “Ti” input is given as a number and indicates the initial time for the
simulation. It is common for all drones and by default set at 0s.

For most modes, the “Tf” input is given as a number and corresponds to the
final time of the simulation “Tf”. It is therefore employed for creating the
trajectories to reach goal position. However, in modes that require regular
updates as “track” or “guided”, it is substituted by the update time. In these
cases, it should be slightly modified within drones. It is usually around 0.5s.

The numerical time step “numTimeStep” is employed for the trajectories.

"""



def full_scenario(Ti,Ts,Tf,ctrlType,trajSelect,numTimeStep):
    pos_obs = np.array([[1, 5, -2], [8, 2, -8], [5, 8, -9], [0, 0, -2], [3, 3, -1],[3, 9, -17],[5, 7, -18],[0, 0, -10],[5, 10, -16],[10,10,-12],[13,13,-13]])
    quad0 = Quadcopter(Ti, Tf, ctrlType, trajSelect, numTimeStep, Ts, quad_id = 0, mode='ennemy', id_targ = -1, color = 'blue', pos_ini = [0,0,0], pos_goal= [15,15,-15], pos_obs = pos_obs)
    quad1 = Quadcopter(Ti, Ts*90, ctrlType, trajSelect, numTimeStep, Ts, quad_id = 1, mode='guided', id_targ = -1, color = 'green', pos_ini = [0,3,0], pos_goal = [15,10,-15], pos_obs = pos_obs)
    quad2 = Quadcopter(Ti, Ts*100, ctrlType, trajSelect, numTimeStep, Ts, quad_id = 2,  mode='track', id_targ = 0, color = 'pink', pos_ini = [3,0,0], pos_goal = [15,20,-15], pos_obs = pos_obs)
    quads = [quad0, quad1, quad2]
    return pos_obs,quads

def multi_waypoint_scenario(Ti,Ts,Tf,ctrlType,trajSelect,numTimeStep):
    pos_obs = np.array([[50,0,0]])
    quad0 = Quadcopter(Ti, Tf, ctrlType, trajSelect, numTimeStep, Ts, quad_id = 0, mode='ennemy', id_targ = -1, color = 'blue', pos_ini = [0,0,0], pos_goal= [0,-17,-10], pos_obs = pos_obs)
    quad1 = Quadcopter(Ti, Tf, ctrlType, trajSelect, numTimeStep, Ts, quad_id = 1, mode='ennemy', id_targ = -1, color = 'green', pos_ini = [20,0,0], pos_goal = [-20,-15,-10], pos_obs = pos_obs)
    quad2 = Quadcopter(Ti, Tf, ctrlType, trajSelect, numTimeStep, Ts, quad_id = 2, mode='ennemy', id_targ = -1, color = 'red', pos_ini = [-20,-10,0], pos_goal = [-10,0,-20], pos_obs = pos_obs)
    quads = [quad0, quad1, quad2]
    return pos_obs,quads

def static_OA_scenario(Ti,Ts,Tf,ctrlType,trajSelect,numTimeStep):
    pos_obs = []
    for i in range(30):
        pos_obs.append(random.sample(range(-10, 0), 3))
    pos_obs = np.array(pos_obs)
    print(pos_obs)
    quad0 = Quadcopter(Ti, Tf, ctrlType, trajSelect, numTimeStep, Ts, quad_id = 0, mode='ennemy', id_targ = -1, color = 'blue', pos_ini = [0,0,0], pos_goal= [-10,-10,-10], pos_obs = pos_obs)
    quads = [quad0]
    return pos_obs,quads

def dynamic_CA_scenario(Ti,Ts,Tf,ctrlType,trajSelect,numTimeStep):
    #Tf =8s
    pos_obs = np.array([[50,0,0]])
    quad0 = Quadcopter(Ti, Ts*100, ctrlType, trajSelect, numTimeStep, Ts, quad_id = 0, mode='guided', id_targ = -1, color = 'blue',  pos_ini = [0,10,-5],pos_goal = [30,10,-5], pos_obs = pos_obs)
    quad1 = Quadcopter(Ti, Tf, ctrlType, trajSelect, numTimeStep, Ts, quad_id = 1, mode='ennemy', id_targ = -1, color = 'green', pos_ini = [3,0,-5], pos_goal = [3,20,-5], pos_obs = pos_obs)
    quad2 = Quadcopter(Ti, Tf, ctrlType, trajSelect, numTimeStep, Ts, quad_id = 2, mode='ennemy', id_targ = -1, color = 'green', pos_ini = [8,0,-5], pos_goal = [8,20,-5], pos_obs = pos_obs)
    quad3 = Quadcopter(Ti, Tf, ctrlType, trajSelect, numTimeStep, Ts, quad_id = 3, mode='ennemy', id_targ = -1, color = 'green', pos_ini = [15,0,-5], pos_goal = [15,20,-5], pos_obs = pos_obs)
    quads = [quad0, quad1,quad2,quad3]
    return pos_obs,quads

def dynamic_CA_scenario_random_pos(Ti,Ts,Tf,ctrlType,trajSelect,numTimeStep):
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

def simple_tracking_scenario(Ti,Ts,Tf,ctrlType,trajSelect,numTimeStep):
    pos_obs = np.array([[-10,-10,0]])
    quad0 = Quadcopter(Ti, Tf, ctrlType, trajSelect, numTimeStep, Ts, quad_id = 0, mode='ennemy', id_targ = -1, color = 'blue',  pos_ini = [0,0,0],  pos_goal = [15,15,-15],  pos_obs = pos_obs)
    quad1 = Quadcopter(Ti, Ts*90, ctrlType, trajSelect, numTimeStep, Ts, quad_id = 1, mode='track', id_targ = 0, color = 'green', pos_ini = [5,5,0], pos_goal = [2,2,-10],  pos_obs = pos_obs)
    quads = [quad0, quad1]
    return pos_obs,quads

def multi_tracking_scenario(Ti,Ts,Tf,ctrlType,trajSelect,numTimeStep):
    pos_obs = np.array([[-10,-10,0]])
    quad0 = Quadcopter(Ti, Tf, ctrlType, trajSelect, numTimeStep, Ts, quad_id = 0, mode='ennemy', id_targ = -1, color = 'blue',  pos_ini = [0,0,0],  pos_goal = [15,15,-15],  pos_obs = pos_obs)
    quad1 = Quadcopter(Ti, Ts*100, ctrlType, trajSelect, numTimeStep, Ts, quad_id = 1, mode='track', id_targ = 0, color = 'green', pos_ini = [4,0,0], pos_goal = [4,4,-10],  pos_obs = pos_obs)
    quad2 = Quadcopter(Ti, Ts*100, ctrlType, trajSelect, numTimeStep, Ts, quad_id = 2, mode='track', id_targ = 0, color = 'green', pos_ini = [4,4,0], pos_goal = [4,4,-10],  pos_obs = pos_obs)
    quad3 = Quadcopter(Ti, Ts*100, ctrlType, trajSelect, numTimeStep, Ts, quad_id = 3, mode='track', id_targ = 0, color = 'green', pos_ini = [4,-4,0], pos_goal = [4,4,-10],  pos_obs = pos_obs)
    quads = [quad0, quad1, quad2, quad3]
    return pos_obs,quads

def tracking_loop_scenario(x,Ti,Ts,Tf,ctrlType,trajSelect,numTimeStep):
    pos_obs = np.array([[x/2,x/2,-10]])
    quad0 = Quadcopter(Ti, Ts*99, ctrlType, trajSelect, numTimeStep, Ts, quad_id = 0, mode='track', id_targ = 1, color = 'blue',  pos_ini = [0,0,-10],  pos_goal = [0,x,-10],  pos_obs = pos_obs)
    quad1 = Quadcopter(Ti, Ts*100, ctrlType, trajSelect, numTimeStep, Ts, quad_id = 1, mode='track', id_targ = 2, color = 'green', pos_ini = [x,0,-10], pos_goal = [0,0,-10],  pos_obs = pos_obs)
    quad2 = Quadcopter(Ti, Ts*101, ctrlType, trajSelect, numTimeStep, Ts, quad_id = 2, mode='track', id_targ = 3, color = 'orange', pos_ini = [x,x,-10],pos_goal = [x,0,-10], pos_obs = pos_obs)
    quad3 = Quadcopter(Ti, Ts*102, ctrlType, trajSelect, numTimeStep, Ts, quad_id = 3, mode='track', id_targ = 0, color = 'pink', pos_ini = [0,x,-10], pos_goal = [x,x,-10],pos_obs = pos_obs)
    quads = [quad0, quad1,quad2,quad3]
    return pos_obs,quads

def tracking_and_kill_scenario(Ti,Ts,Tf,ctrlType,trajSelect,numTimeStep):
    pos_obs = np.array([[-10,-10,0]])
    quad0 = Quadcopter(Ti, Tf, ctrlType, trajSelect, numTimeStep, Ts, quad_id = 0, mode='ennemy', id_targ = -1, color = 'blue',  pos_ini = [0,0,-5],  pos_goal = [20,15,-20],  pos_obs = pos_obs)
    quad1 = Quadcopter(Ti, Ts*100, ctrlType, trajSelect, numTimeStep, Ts, quad_id = 1, mode='track', id_targ = 0, color = 'green', pos_ini = [5,0,0], pos_goal = [4,4,-10],  pos_obs = pos_obs)
    quads = [quad0, quad1]
    return pos_obs,quads

def simple_guided_for_PF(Ti,Ts,Tf,ctrlType,trajSelect,numTimeStep):
    pos_obs = []
    for i in range(20):
        pos_obs.append(random.sample(range(-10, 0), 3))
    pos_obs = np.array(pos_obs)
    quad0 = Quadcopter(Ti, Ts*100, ctrlType, trajSelect, numTimeStep, Ts, quad_id = 0, mode='guided', id_targ = -1, color = 'blue',  pos_ini = [0,0,-5],  pos_goal = [-10,-10,-10],  pos_obs = pos_obs)
    quads = [quad0]
    return pos_obs,quads

def ROS_simu(Ti,Ts,Tf,ctrlType,trajSelect,numTimeStep):
    fire_station=[]
    fire_truck=[]
    tree_1=[]
    tree_2=[]
    pos_obs=[]
    for i in range(20):
        x = random.sample(range(-10, 10), 1)[0]
        y = random.sample(range(-55, -45), 1)[0]
        z = random.sample(range(-12, 0), 1)[0]
        fire_station.append([x,y,z])
    
    for i in range(5):
        x = random.sample(range(-19, 21), 1)[0]
        y = random.sample(range(-55, -45), 1)[0]
        z = random.sample(range(-3, 0), 1)[0]
        fire_truck.append([x,y,z])

    for i in range(5):
        x = random.sample(range(-12, -8), 1)[0]
        y = random.sample(range(-42,-38), 1)[0]
        z = random.sample(range(-5, 0), 1)[0]
        tree_1.append([x,y,z])
    for i in range(5):
        x = random.sample(range(8, 12), 1)[0]
        y = random.sample(range(-42,-38), 1)[0]
        z = random.sample(range(-5, 0), 1)[0]
        tree_2.append([x,y,z])

    pos_obs = fire_station + fire_truck + tree_1 + tree_2
    pos_obs = np.array(pos_obs)
    quad0 = Quadcopter(Ti, Ts*100, ctrlType, trajSelect, numTimeStep, Ts, quad_id = 0, mode='guided', id_targ = -1, color = 'blue',  pos_ini = [0,0,0],  pos_goal = [0,-100,-10],  pos_obs = pos_obs)
    quads = [quad0]
    return(pos_obs,quads)

def real_map(Ti,Ts,Tf,ctrlType,trajSelect,numTimeStep):

    xs = [-1,0,1]
    ys = [-1,0,1]
    zs = [0,-1,-2,-3,-4,-5,-6,-7,-8,-9,-10]
    tower = [[x,y,z] for x in xs for y in ys for z in zs]

    xs = [-20,5,10]
    ys = [5,-10,10]
    zs = [0,-1,-2,-3]
    trees = [[x,y,z] for x in xs for y in ys for z in zs]

    xs = [-20,5,10]
    ys = [5,-10,10]
    zs = [-4,-5]

    tops = []
    for i in range(3):
        x, y = xs[i], ys[i]
        for z in zs:
            tops = tops + [[x-1,y,z],[x+1,y,z],[x,y,z],[x,y-1,z],[x,y+1,z]]
            print(tops)

    pos_obs = np.array(tower + trees + tops)

    quad0 = Quadcopter(Ti, Tf, ctrlType, trajSelect, numTimeStep, Ts, quad_id = 0, mode='ennemy', id_targ = -1, color = 'blue',  pos_ini = [0,0,-5],  pos_goal = [-10,-10,-10],  pos_obs = pos_obs)
    quads = [quad0]
    return pos_obs,quads
