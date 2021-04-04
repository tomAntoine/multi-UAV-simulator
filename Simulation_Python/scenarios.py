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
