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



def main():
    start_time = time.time()

    # Simulation Setup
    # ---------------------------
    Ti = 0
    Ts = 0.005
    Tf = 15
    ifsave = 0

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

    def dynamic_avoidance_scenario():
        pos_obs = np.array([[50,0,0]])
        quad0 = Quadcopter(Ti, Tf, ctrlType, trajSelect, numTimeStep, Ts, quad_id = 0, mode='ennemy', id_targ = -1, color = 'blue', pos_ini = [0,14,-5], pos_goal= [0,-20,-5], pos_obs = pos_obs)
        quad1 = Quadcopter(Ti, Ts*90, ctrlType, trajSelect, numTimeStep, Ts, quad_id = 1, mode='guided', id_targ = -1, color = 'green', pos_ini = [20,0,-5], pos_goal = [-20,0,-5], pos_obs = pos_obs)
        quads = [quad0, quad1]
        return pos_obs,quads

    pos_obs,quads = dynamic_avoidance_scenario()


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

        if t>t_int:
            print("{0}s over {1}s".format(t_int, Tf))
            t_int = int(t)+1

        t += Ts
        i += 1

    end_time = time.time()
    print("Simulated {:.2f}s in {:.6f}s.".format(t, end_time - start_time))

    # View Results
    # ---------------------------

    data = dict()

    for quad in quads:
        data[str(quad.quad_id)] = dict([('mode',quad.mode_ini),
                                        ('id_targ',quad.id_targ),
                                        ('waypoints',quad.traj.wps_pf3d),
                                        ('color',quad.color),
                                        ('t_all',quad.t_all),
                                        ('s_all',quad.s_all),
                                        ('pos_goal', quad.pos_goal_ini),
                                        ('pos_all',quad.pos_all),
                                        ('vel_all',quad.vel_all),
                                        ('quat_all',quad.quat_all),
                                        ('omega_all',quad.omega_all),
                                        ('euler_all',quad.euler_all),
                                        ('sDes_traj_all',quad.sDes_traj_all),
                                        ('sDes_calc_all',quad.sDes_calc_all),
                                        ('w_cmd_all',quad.w_cmd_all),
                                        ('wMotor_all',quad.wMotor_all),
                                        ('thr_all',quad.thr_all),
                                        ('tor_all',quad.tor_all),
                                        ('params', quad.params),
                                        ('xyzType', quad.traj.xyzType),
                                        ('yawType', quad.traj.yawType)])



    utils.makeAllFigures(data, pos_obs)
    #utils.make3DAnimation(data,pos_obs)


    fig = plt.figure(figsize = (10,10))

    ax = p3.Axes3D(fig)

    anis = []
    '''
    for quad in quads :
    ax.plot([quad.pos_ini[0]], [quad.pos_ini[1]], -quad.pos_ini[2], 's', color=quad.color)
    ax.plot([quad.pos_goal[0]],[quad.pos_goal[1]], -quad.pos_goal[2],'X', color=quad.color)
    #ani = utils.sameAxisAnimation(quad.t_all, quad.traj.wps, quad.pos_all, quad.quat_all, quad.sDes_traj_all, quad.Ts, quad.params, quad.traj.xyzType, quad.traj.yawType, ifsave, ax, fig, quad.color, quad.mode_ini, quad.quad_id, quad.id_targ)
    '''
    ani = utils.sameAxisAnimation(quads[0].t_all, quads[0].Ts, data, ifsave, ax, fig, pos_obs)
    '''
    anis.append(ani)
    if quad.mode_ini == 'track':
        sentence = 'Quad ID: {}, MODE: {}, TARGET'.format(quad.quad_id,quad.mode_ini,quad.id_targ)
    elif quad.mode_ini != 'track':
        sentence = 'Quad ID: {}, MODE: {}'.format(quad.quad_id,quad.mode_ini)
    ax.legend(sentence)
    '''



    '''
    # Plot pf3D potential
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ani = utils.pfPlot(quads[0].t_all,quads[0].pf_map_all,ax,fig,Ts)
    '''
    plt.show()

if __name__ == "__main__":
    if (config.orient == "NED" or config.orient == "ENU"):
        main()
        # cProfile.run('main()')
    else:
        raise Exception("{} is not a valid orientation. Verify config.py file.".format(config.orient))
