# -*- coding: utf-8 -*-
"""
author: John Bass
email: john.bobzwik@gmail.com
license: MIT
Please feel free to use and modify this, but keep the above information. Thanks!
"""

import numpy as np
from numpy import sin, cos, tan, pi, sign
from scipy.integrate import ode

import utils
import config

from pf3d import pf3d

from quadFiles.ROSQuad import *

deg2rad = pi/180.0


class Quadcopter:

    def __init__(self, Tup, Ts, quad_id = None, mode = 'ennemy', id_targ = -1, pos_goal= [1,1,-1], pos_obs = None, channel_id = None):

        # Quad Params
        # ---------------------------

        self.quad_id = quad_id


        self.pos_goal = pos_goal
        self.pos_obs = pos_obs

        self.id_targ = id_targ

        self.pos_quads = None

        self.mode = mode
        self.mode_ini = mode
        self.neutralize = False

        self.Ts = Ts
        self.t_track = 0
        self.Tup = Tup

        #-- Setup the commanded flying speed
        self.gnd_speed = 4 # [m/s]

        self.channel_id = channel_id


        #-- Connect to the vehicle
        print('Connecting the drone {0}'.format(self.channel_id))


        #channel_id = 'udp:127.0.0.1:14551'

        self.vehicle = connect(channel_id)

        self.global_frame = self.vehicle.location.global_relative_frame

        self.pos_ini = reverse_get_location_metres(self.global_frame, self.vehicle.location.global_relative_frame)

        self.vehicle.groundspeed = self.gnd_speed




    def print_mode(self, mode_prev, mode_new):
        print("Drone {0} switch from {1} to {2}.".format(self.quad_id, mode_prev, mode_new))


    def updateWaypoints2ROS(self):
        #clear_all_mission(self.vehicle)
        N_wps_before, _ = get_current_mission(self.vehicle)

        new_wps = self.wps
        print("new wps are : ----------------")
        for i in range(10):
            wp = new_wps[i+1]
            x, y, z = wp[0], wp[1], -wp[2]
            wp = get_location_metres(self.global_frame,x,y,z)
            print(x,y,z)
            add_last_waypoint_to_mission(self.vehicle, wp.lat, wp.lon, wp.alt)
        add_last_waypoint_to_mission(self.vehicle, wp.lat, wp.lon, wp.alt)
        print(N_wps_before)
        self.vehicle.commands.next = N_wps_before
        ChangeMode(self.vehicle,"AUTO")



    def update(self, Ts):

        self.update_states()

        if self.mode == "fall":

            self.pos_goal = np.hstack([self.pos[0], self.pos[1], -0.5]).astype(float)
            ChangeMode(self.vehicle,"Stabilize")
            self.mode = "neutralized"

            self.print_mode("ennemy", "fall")


        if (self.mode != 'ennemy') :

            if self.mode == 'takeoff':
                arm_and_takeoff(3)
                self.mode = 'track'

                if self.mode == 'track':
                    pos_dyn_obs = [x[1] for x in self.pos_quads if  (x[0] != self.id_targ and x[0] != self.quad_id)]
                    try :
                        temp_pos_obs = np.vstack((self.pos_obs, pos_dyn_obs)).astype(float)
                    except :
                        temp_pos_obs = np.vstack((self.pos_obs)).astype(float)
                    pos_targ = [x[1] for x in self.pos_quads if x[0] == self.id_targ][0]

                    # mix desired velocity vector with the velocity of the target
                    # lectures on coop guidance P16 ==> modif for moving target (it will add a gain)
                    # need to understand the guidance command we are using
                    # derivative ==> more reactivness
                    # otherwise moddif wp but stab issues
                    # ask SA if they can provide us the velocity of the targets, better than discretize estimation

                    estimate_pos_targ = 2*pos_targ-self.previous_pos_targ


                    self.pos_goal = np.hstack((estimate_pos_targ) + [0,0,-0.5]).astype(float)
                    dist = np.sqrt((self.pos[0]-self.pos_goal[0])**2+(self.pos[1]-self.pos_goal[1])**2+(self.pos[2]-self.pos_goal[2])**2)
                    if dist < 1:
                        self.t_track += Ts
                        if self.t_track > 3*Ts:
                            self.neutralize = True
                            self.mode = "home"
                            self.print_mode("track", "home")
                    else :
                        self.t_track = 0
                    self.previous_pos_targ = pos_targ


                if self.mode == 'guided':
                    pos_dyn_obs = [x[1] for x in self.pos_quads if  (x[0] != self.quad_id)]
                    try :
                        temp_pos_obs = np.vstack((self.pos_obs, pos_dyn_obs)).astype(float)
                    except :
                        temp_pos_obs = np.vstack((self.pos_obs)).astype(float)

                    dist = np.sqrt((self.pos[0]-self.pos_goal[0])**2+(self.pos[1]-self.pos_goal[1])**2+(self.pos[2]-self.pos_goal[2])**2)
                    if dist < 1:
                        self.mode = "home"
                        self.print_mode("guided", "home")

                if self.mode == 'home':
                    ChangeMode(self.vehicle,"RTL")


                if self.mode == 'land':
                    pos_dyn_obs = [x[1] for x in self.pos_quads if  (x[0] != self.quad_id)]
                    try :
                        temp_pos_obs = np.vstack((self.pos_obs, pos_dyn_obs)).astype(float)
                    except :
                        temp_pos_obs = np.vstack((self.pos_obs)).astype(float)
                    self.pos_goal = np.hstack([self.pos[0], self.pos[1], -0.5]).astype(float)

                if self.mode == 'hover':
                    try :
                        temp_pos_obs = np.vstack((self.pos_obs, pos_dyn_obs)).astype(float)
                    except :
                        temp_pos_obs = np.vstack((self.pos_obs)).astype(float)
                    temp_pos_obs = np.vstack((self.pos_obs, pos_dyn_obs)).astype(float)
                    self.pos_goal = np.hstack(self.pos).astype(float)


        self.wps, data = pf3d(self.pos, self.pos_goal, self.pos_obs)
        self.data = data
        self.updateWaypoints2ROS()

    def update_states(self):

        self.pos = reverse_get_location_metres(self.global_frame, self.vehicle.location.global_relative_frame)
        
        self.speed = self.vehicle.velocity

        print(self.pos)
        print(self.speed)


    def init(self):

        arm_and_takeoff(self.vehicle,3)
        self.update_states()
        self.wps = self.pos
        clear_all_mission(self.vehicle)
        x, y, z = self.wps[0], self.wps[1], self.wps[2]
        wp = get_location_metres(self.global_frame,x,y,z)
        print(wp)
        add_last_waypoint_to_mission(self.vehicle, wp.lat, wp.lon, wp.alt)
        ChangeMode(self.vehicle,"AUTO")