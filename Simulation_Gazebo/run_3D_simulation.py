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
import time
from quadFiles.quad import Quadcopter
import utils

def quad_sim(Ts, quads):

    pos_quads = []

    for quad in quads:
        quad.update_states()

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

    Ts = 0.005 # final time of simulation
    Tc = time.time() # clock time of simulation

    # ---------------------------
    pos_obs = np.array([[1, 5, -2]])

          
    quad0 = Quadcopter(Ts = Ts, quad_id = 0, mode='ennemy', id_targ = -1, pos_goal= [30,15,-30], pos_obs = pos_obs, channel_id='udp:127.0.0.1:14551', global_frame=None)
    # Initialize the frame at the origin by initializing the drone located at the origin
    #global_frame = quad0.global_frame
    #quad1 = Quadcopter(Ts = Ts, quad_id = 1, mode='guided', id_targ = -1, pos_goal= [15,10,-15], pos_obs = pos_obs, channel_id='udp:127.0.0.1:14561', global_frame=global_frame)
    #quad2 = Quadcopter(Ts = Ts, quad_id = 2, mode='track', id_targ = 0, pos_goal= [15,20,-15], pos_obs = pos_obs, channel_id='udp:127.0.0.1:14571', global_frame=global_frame)
    

    quads = [quad0]#, quad1, quad2]

    for quad in quads:

        quad.init()

    while True :


        quad_sim(Ts, quads)

        time.sleep(3)



if __name__ == "__main__":
    print("simulation started")
    main()
