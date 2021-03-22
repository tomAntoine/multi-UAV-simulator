import numpy as np
from numpy import sin, cos, tan, pi, sign
from scipy.integrate import ode

from pf3d import pf3d

from quadFiles.ROSQuad import *

import time

deg2rad = pi/180.0


gnd_speed = 4 # [m/s]

channel_id = 'udp:127.0.0.1:14551'


#-- Connect to the vehicle
print('Connecting the drone {0}'.format(channel_id))


#channel_id = 'udp:127.0.0.1:14551'

vehicle = connect(channel_id)

global_frame = vehicle.location.global_relative_frame

pos_ini = reverse_get_location_metres(global_frame, vehicle.location.global_relative_frame)
print(pos_ini)

vehicle.groundspeed = gnd_speed

arm_and_takeoff(vehicle,3)

def update_states(vehicle):

    pos = reverse_get_location_metres(global_frame, vehicle.location.global_relative_frame)
    speed = vehicle.velocity
    return pos, speed



pos, speed = update_states(vehicle)
wps = pos
clear_all_mission(vehicle)
x, y, z = wps[0], wps[1], wps[2]
print (wps)
wp = get_location_metres(global_frame,x-3,y,z)
wp2 = get_location_metres(global_frame,x-3,y-3,z+1)

print(wp)

add_last_waypoint_to_mission(vehicle, wp.lat, wp.lon, wp.alt)
add_last_waypoint_to_mission(vehicle, wp2.lat, wp2.lon, wp2.alt)
ChangeMode(vehicle,"AUTO")

#time.sleep(1)

clear_all_mission(vehicle)

wp = get_location_metres(global_frame,x+10,y,z+10)
wp2 = get_location_metres(global_frame,x+10,y+10,z+21)
wp3 = get_location_metres(global_frame,x+20,y+20,z+31)


add_last_waypoint_to_mission(vehicle, wp.lat, wp.lon, wp.alt)
add_last_waypoint_to_mission(vehicle, wp2.lat, wp2.lon, wp2.alt)
add_last_waypoint_to_mission(vehicle, wp3.lat, wp3.lon, wp3.alt)
add_last_waypoint_to_mission(vehicle, wp2.lat, wp2.lon, wp2.alt)
ChangeMode(vehicle,"GUIDED")
ChangeMode(vehicle,"AUTO")
