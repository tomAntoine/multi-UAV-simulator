# -*- coding: utf-8 -*-
"""
author: John Bass
email: john.bobzwik@gmail.com
license: MIT
Please feel free to use and modify this, but keep the above information. Thanks!
"""
# Functions get_poly_cc, minSomethingTraj, pos_waypoint_min are derived from Peter Huang's work:
# https://github.com/hbd730/quadcopter-simulation
# author: Peter Huang
# email: hbd730@gmail.com
# license: BSD
# Please feel free to use and modify this, but keep the above information. Thanks!


from pf3d import pf3d

class Trajectory:

    def __init__(self, psi, pos_ini=[0,0,0], pos_goal=[20,20,20],pos_obs = None,Tf=18,t_ini=0):
        """
        self.ctrlType = ctrlType
        self.xyzType = trajSelect[0]
        self.yawType = trajSelect[1]
        self.averVel = trajSelect[2]

        #self.wps_pf2d = pf2d(pos_ini, pos_goal, pos_obs)
        #self.wps = makeWaypoints(self.wps_pf2d)
        """
        self.wps, data = pf3d(pos_ini, pos_goal, pos_obs)
        self.data = data
