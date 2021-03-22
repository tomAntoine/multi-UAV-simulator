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
import mpl_toolkits.mplot3d.axes3d as p3

def quad_sim(Ts, quads):

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

        quad.update(Ts)

def main():

    Ts = 0.005

    # ---------------------------
    pos_obs = np.array([[1, 5, -2], [8, 2, -8], [5, 8, -9], [0, 0, -2], [3, 3, -1],[3, 9, -17],[5, 7, -18],[0, 0, -10],[5, 10, -16],[10,10,-12],[13,13,-13]])


    pos_goal = [15,15,-15]
    quad0 = Quadcopter(Tup = 100, Ts = Ts, quad_id = 0, mode='ennemy', id_targ = -1, pos_goal= pos_goal, pos_obs = pos_obs)
    
    quads = [quad0]

    for quad in quads:
        
        quad.init()

    while True :

        quad_sim(Ts, quads)
        
        time.sleep(Ts)



if __name__ == "__main__":
    main()