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

    Ts = 15

    # ---------------------------
    pos_obs = np.array([[1, 5, -2]])

    pos_goal = [15,15,-15]
    channel_id = 'udp:127.0.0.1:14551'
    
    quad0 = Quadcopter(Tup = 100, Ts = Ts, quad_id = 0, mode='ennemy', id_targ = -1, pos_goal= pos_goal, pos_obs = pos_obs,channel_id=channel_id)
    

    quads = [quad0]

    for quad in quads:

        quad.init()

    while True :

        quad_sim(Ts, quads)

        time.sleep(Ts)



if __name__ == "__main__":
    print("simulation started")
    main()