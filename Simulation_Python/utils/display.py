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
from numpy import pi
import matplotlib.pyplot as plt
import utils
import matplotlib.collections as collections


rad2deg = 180.0/pi
deg2rad = pi/180.0
rads2rpm = 60.0/(2.0*pi)
rpm2rads = 2.0*pi/60.0

# Print complete vector or matrices
def fullprint(*args, **kwargs):
    opt = np.get_printoptions()
    np.set_printoptions(threshold=np.inf)
    print(*args, **kwargs)
    np.set_printoptions(opt)

def dist(a,b):
    return np.sqrt((a[0]-b[0])**2+(a[1]-b[1])**2+(a[2]-b[2])**2)

def makeAllFigures(data, pos_obs):

    time = data["0"]["t_all"]

    id_list = []
    for id_quad in data:
        id_list.append(id_quad)

    # Waypoint accuracy

    for id_quad in (data):

        pos = data[id_quad]["pos_all"]
        dist2way_id = []

        for i in range(len(pos)):
            way = data[id_quad]["pos_goal"]
            dist2way_id.append(dist(way,pos[i,:]))

        data[id_quad]['dist2way'] = dist2way_id
       

    # Obstacle avoidance

    for id_quad in (data):

        pos = data[id_quad]["pos_all"]
        dist2obs_id = []

        for i in range(len(pos)):

            dist2obs_i = []
            for obs in pos_obs :

                dist2obs_i.append(dist(obs,pos[i,:]))

            dist2obs_id.append(min(dist2obs_i))

        data[id_quad]['dist2obs'] = dist2obs_id

    
    # Collision avoidance

    for id_quad in (data):

        pos = data[id_quad]["pos_all"]
        dist2agt_id = []

        for i in range(len(pos)):

            dist2agt_i = []

            for id_quad_2 in (data):

                if (id_quad != id_quad_2) and (id_quad_2 != data[id_quad]["id_targ"]) :

                    pos_2 = data[id_quad_2]["pos_all"]

                    dist2agt_i.append(dist(pos_2[i,:],pos[i,:]))

            dist2agt_id.append(min(dist2agt_i))

        data[id_quad]['dist2agt'] = dist2agt_id

    # Traking performance

    id_trackers = []

    for id_quad in (data):

        if (data[id_quad]["mode"] == "track") :

            id_trackers.append(id_quad)            

            pos = data[id_quad]["pos_all"]
            id_target = str(data[id_quad]["id_targ"])
            pos_2 = data[id_target]["pos_all"]

            dist2target = []

            for i in range(len(pos)):

                dist2target.append(dist(pos[i,:],pos_2[i,:]))

            data[id_quad]['dist2target'] = dist2target
    


    ### plots

    plt.show()

    plt.figure()
    for id_quad in (data):
        # Prepare the legend by agents and ennemies depending on modes
        if data[id_quad]["mode"] == 'track':
            sentence = 'Quad ID: {}, MODE: {}, TARGET: Quad {}'.format(id_quad,data[id_quad]["mode"],data[id_quad]["id_targ"])
        elif data[id_quad]["mode"] != 'track':
            sentence = 'Quad ID: {}, MODE: {}'.format(id_quad,data[id_quad]["mode"])
        id_plot = plt.plot(time, data[id_quad]["dist2way"], color = data[id_quad]["color"], label = sentence)
    min_dist = 100
    for id_quad in (data):
        min_dis_i = min(data[id_quad]["dist2way"])
        if min_dis_i < min_dist:
            min_dist = min_dis_i
    sentence = 'Final Distance to Waypoint {} m'.format(round(min_dist,3))
    min_dist_obj = plt.plot(time,[min_dist for t in time], '--', color = 'red', label = sentence)
    plt.grid(True)
    plt.legend()
    plt.xlabel('Time (s)')
    plt.ylabel('Distance (m)')
    plt.title('Distance to Waypoint')    
    plt.draw()
    #plt.savefig('ObstacleAvoidance/Sim5_wp.png', dpi=80)


    fig = plt.figure()
    for id_quad in (data):
        # Prepare the legend by agents and ennemies depending on modes
        if data[id_quad]["mode"] == 'track':
            sentence = 'Quad ID: {}, MODE: {}, TARGET: Quad {}'.format(id_quad,data[id_quad]["mode"],data[id_quad]["id_targ"])
        elif data[id_quad]["mode"] != 'track':
            sentence = 'Quad ID: {}, MODE: {}'.format(id_quad,data[id_quad]["mode"])
        id_plot = plt.plot(time, data[id_quad]["dist2agt"], color = data[id_quad]["color"], label = sentence)
    min_dist = 100
    for id_quad in (data):
        min_dis_i = min(data[id_quad]["dist2agt"])
        if min_dis_i < min_dist:
            min_dist = min_dis_i

    sentence = 'Minimum distance {} m'.format(round(min_dist,3))
    min_dist_obj = plt.plot(time,[min_dist for t in time], '--', color = 'red', label = sentence)

    plt.grid(True)
    plt.legend()
    plt.xlabel('Time (s)')
    plt.ylabel('Distance (m)')
    plt.title('Minimum Distance to Other Agents')    
    plt.draw()







    plt.figure()
    for id_quad in (data):
        # Prepare the legend by agents and ennemies depending on modes
        if data[id_quad]["mode"] == 'track':
            sentence = 'Quad ID: {}, MODE: {}, TARGET: Quad {}'.format(id_quad,data[id_quad]["mode"],data[id_quad]["id_targ"])
        elif data[id_quad]["mode"] != 'track':
            sentence = 'Quad ID: {}, MODE: {}'.format(id_quad,data[id_quad]["mode"])
        id_plot = plt.plot(time, data[id_quad]["dist2obs"], color = data[id_quad]["color"], label = sentence)
    min_dist = 100
    for id_quad in (data):
        min_dis_i = min(data[id_quad]["dist2obs"])
        if min_dis_i < min_dist:
            min_dist = min_dis_i
    sentence = 'Minimum distance {} m'.format(round(min_dist,3))
    min_dist_obj = plt.plot(time,[min_dist for t in time], '--', color = 'red', label = sentence)
    plt.grid(True)
    plt.legend()
    plt.xlabel('Time (s)')
    plt.ylabel('Distance (m)')
    plt.title('Minimum Distance to Obstacle')    
    plt.draw()
    #plt.savefig('ObstacleAvoidance/Sim5_oa.png', dpi=80)


    if id_trackers :

        fig = plt.figure()
        for id_quad in (id_trackers):
            # Prepare the legend by agents and ennemies depending on modes
            sentence = 'Quad ID: {}, MODE: {}, TARGET: Quad {}'.format(id_quad,data[id_quad]["mode"],data[id_quad]["id_targ"])
            id_plot = plt.plot(time, data[id_quad]["dist2target"], color = data[id_quad]["color"], label = sentence)
        plt.grid(True)
        sentence = 'Threshold distance: 1 m'
        threshold = plt.plot(time,[0.9 for t in time], '--', color = 'red', label = sentence)
        plt.legend()
        plt.xlabel('Time (s)')
        plt.ylabel('Distance (m)')
        plt.title('Minimum Distance to Target')    
        plt.draw()

        time_killed = 10 #data[id_quad]["t_track"]-50*time[1]


        sentence = 'Time for neutralization {} s'.format(round(time_killed,3))
        min_dist_obj = plt.vlines(time_killed,0,8, color = 'red', label = sentence)
        
        xrange1 = [(0, time_killed)]
        xrange2 = [(time_killed, time[-1])]
        yrange = (0, 10)
        ax = fig.add_subplot(111)
        c1 = collections.BrokenBarHCollection(xrange1, yrange, facecolor='pink', alpha=0.2)
        c2 = collections.BrokenBarHCollection(xrange2, yrange, facecolor='blue', alpha=0.2)

        ax.add_collection(c1)
        ax.add_collection(c2)
