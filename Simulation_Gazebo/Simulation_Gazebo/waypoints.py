# -*- coding: utf-8 -*-
"""
author: John Bass
email: john.bobzwik@gmail.com
license: MIT
Please feel free to use and modify this, but keep the above information. Thanks!
"""

import numpy as np
from numpy import pi
import config

deg2rad = pi/180.0

def makeWaypoints(wps,t_ini=0,Tf=18):

    yaw_ini = 0
    v_average = 3

    t = np.linspace(t_ini, Tf, num=len(wps))
    yaw = np.array([0.0 for k in range(len(t))])

    t = np.hstack((t)).astype(float)
    wps = np.vstack((wps)).astype(float)
    yaw = np.hstack((yaw)).astype(float)*deg2rad

    return t, wps, yaw, v_average


