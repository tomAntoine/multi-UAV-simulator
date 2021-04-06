# -*- coding: utf-8 -*-
"""
Adapted by:
Tom Antoine and Alejandra Martínez
part of GNC subteam, group 1, GDP AVDC 2020-2021
email:
tom.antoine@cranfield.ac.uk
alejandra.martinez-farina@cranfield.ac.uk

Based on a code by:
author: John Bass
email: john.bobzwik@gmail.com
github: https://github.com/bobzwik/Quadcopter_SimCon
license: MIT


Please feel free to use and modify this, but keep the above information. Thanks!
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
from scenarios import *


def quad_sim(t, Ts, quads, wind, i):
    """
    this function update the states of the drones
    it is composed of to loops
    the first one collect the position of every drones in a list nammed pos_quads
    then, in the second loop, this list is provided to every drone, 
    and their states are updated regarding the positions of others, to their tasks, trajectories, to the time, etc
    """


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
    """
    this main function is exectued each time the file is executed
    it first define the quads and the obstacles in the simulation,
    then it compute the simulation at any time
    finally, plots and results are returned, with a possibility of saving the results
    the data dictionary is created to store all the usefull data from the simulation
    """

    start_time = time.time()
    

    # the desired scenario should be selected regarding the file scenarios.py
    # it is also possible to define position of obstacles and the drones here

    pos_obs,quads = static_OA_scenario(Ti,Ts,Tf,ctrlType,trajSelect,numTimeStep)

    wind = Wind('None', 2.0, 90, -15)

    for quad in quads:
        quad.init(Ts,quad)

    # Run Simulation
    # ---------------------------
    t = Ti
    i = 1
    t_int = 0

    # here, the loop is run for the time of the simulation.
    # at each step, the function quad_sim is called
    # this function update the states of the drones according to the new time 

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
                                        ('yawType', quad.traj.yawType),
                                        ('t_track', quad.t_track)])
    # Plot analysis
    #utils.makeAllFigures(data, pos_obs)
    #utils.make3DAnimation(data,pos_obs)

    # Plot pf3D potential
    #fig = plt.figure()
    #ax = fig.add_subplot(111, projection='3d')
    #ani = utils.pfPlot(quads[0].t_all,quads[0].pf_map_all,ax,fig,Ts)

    # 3d animation
    fig = plt.figure(figsize = (13,13))
    ax = p3.Axes3D(fig)
    ani = utils.sameAxisAnimation(quads[0].t_all, quads[0].Ts, data, ifsave, ax, fig, pos_obs)

    plt.show()

if __name__ == "__main__":
    if (config.orient == "NED" or config.orient == "ENU"):

        # Simulation Setup
        # ---------------------------
        Ti = 0
        Ts = 0.005
        Tf = 8
        ifsave = 1

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

        main()
        # cProfile.run('main()')
    else:
        raise Exception("{} is not a valid orientation. Verify config.py file.".format(config.orient))
