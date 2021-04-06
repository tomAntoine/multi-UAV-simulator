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
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib import animation
from matplotlib.legend import Legend

import utils
import config

import matplotlib.lines as mlines

numFrames = 8

def sameAxisAnimation(t_all, Ts, data, ifsave, ax, fig, pos_obs):
    obstacles = ax.plot(pos_obs[:,0], pos_obs[:,1], -pos_obs[:,2],'ro')
    #leg = Legend(ax, obstacles, ['Obstacles'], loc='upper right', fancybox=True, fontsize = 16)
    #ax.add_artist(leg)

    obs_leg = mlines.Line2D([], [], color='red', marker='o',markersize=10, label='Obstacles',linestyle='None')
    ini_leg = mlines.Line2D([], [], color='black', marker='s',markersize=10, label='Initial Position',linestyle='None')
    goal_leg = mlines.Line2D([], [], color='black', marker='X',markersize=10, label='Goal Position',linestyle='None')

    #leg = Legend(ax, handles=[obs_leg, ini_leg, goal_leg], labels = ['Obstacles','Initial Position','Goal Position'],loc='upper right', fancybox=True, fontsize = 12)
    ax.legend(handles=[obs_leg, ini_leg, goal_leg], labels = ['Obstacles','Initial Position','Goal Position'],loc='upper right', fancybox=True, fontsize = 12)
    #ax.add_artist(leg)


    lines1 = []
    lines2 = []
    lines3 = []
    pos_ini_agts = []
    pos_ini_targs = []
    sentence_agts = []
    sentence_targs = []
    for id_quad in (data):
        if data[id_quad]["mode"] == 'track':
            sentence = 'Quad ID: {}, MODE: {}, TARGET: Quad {}'.format(id_quad,data[id_quad]["mode"],data[id_quad]["id_targ"])
        elif data[id_quad]["mode"] != 'track':
            sentence = 'Quad ID: {}, MODE: {}'.format(id_quad,data[id_quad]["mode"])

        ax.plot([data[id_quad]["pos_all"][0,0]], [data[id_quad]["pos_all"][0,1]], -data[id_quad]["pos_all"][0,2], 's', color= data[id_quad]["color"], label=sentence)

        if (data[id_quad]["mode"] == 'guided') or (data[id_quad]["mode"] == 'ennemy'):
            ax.plot([data[id_quad]["pos_goal"][0]], [data[id_quad]["pos_goal"][1]], -data[id_quad]["pos_goal"][2],'X', color=data[id_quad]["color"])

        line1, = ax.plot([], [], [], lw=2, color=data[id_quad]["color"], label='_nolegend_')
        line2, = ax.plot([], [], [], lw=2, color=data[id_quad]["color"], label='_nolegend_')
        line3, = ax.plot([], [], [], '--', lw=1, color=data[id_quad]["color"], label='_nolegend_')
        lines1.append(line1)
        lines2.append(line2)
        lines3.append(line3)

        if (data[id_quad]["mode"] == 'ennemy'):
            pos_ini_targs.append(line1)
            sentence_targs.append(sentence)
        else:
            pos_ini_agts.append(line1)
            sentence_agts.append(sentence)

    try:
        legend_agts = Legend(ax, pos_ini_agts, sentence_agts, loc='lower left', fancybox=True, fontsize = 12)
        ax.add_artist(legend_agts)
    except:
        print('No agents quads')

    try:
        legend_targs = Legend(ax, pos_ini_targs, sentence_targs, loc='lower right', fancybox=True, fontsize = 12 )
        ax.add_artist(legend_targs)
    except:
        print('No target quads')

    xmin, xmax = [], []
    ymin, ymax = [], []
    zmin, zmax = [], []

    for id_quad in (data):
        x = data[id_quad]["pos_all"][:,0]
        y = data[id_quad]["pos_all"][:,1]
        z = data[id_quad]["pos_all"][:,2]

        if (config.orient == "NED"):
            z = -z

        xmin.append(x.min())
        xmax.append(x.max())
        ymin.append(y.min())
        ymax.append(y.max())
        zmin.append(z.min())
        zmax.append(z.max())

        '''
        xDes = sDes_tr_all[:,0]
        yDes = sDes_tr_all[:,1]
        zDes = sDes_tr_all[:,2]

        x_wp = waypoints[:,0]
        y_wp = waypoints[:,1]
        z_wp = waypoints[:,2]
        '''


    # Setting the axes properties
    extraEachSide = 0.5
    maxRange = 0.5*np.array([max(xmax)-min(xmin), max(ymax)-min(ymin), max(zmax)-min(zmin)]).max() + extraEachSide
    mid_x = 0.5*(max(xmax)+min(xmin))
    mid_y = 0.5*(max(ymax)+min(ymin))
    mid_z = 0.5*(max(zmax)+min(zmin))

    ax.set_xlim3d([mid_x-maxRange, mid_x+maxRange])
    ax.set_xlabel('X')
    if (config.orient == "NED"):
        ax.set_ylim3d([mid_y+maxRange, mid_y-maxRange])
    elif (config.orient == "ENU"):
        ax.set_ylim3d([mid_y-maxRange, mid_y+maxRange])
    ax.set_ylabel('Y')
    ax.set_zlim3d([0, mid_z+maxRange])
    ax.set_zlabel('Altitude')

    titleTime = ax.text2D(0.05, 0.95, "", transform=ax.transAxes, fontsize = 16)



    def updateLines(i):

        ax.view_init(elev=10.+i/20, azim=i/4)

        time = t_all[i*numFrames]

        for j, id_quad in enumerate(data):
            x = data[id_quad]["pos_all"][i*numFrames,0]
            y = data[id_quad]["pos_all"][i*numFrames,1]
            z = data[id_quad]["pos_all"][i*numFrames,2]

            x_from0 = data[id_quad]["pos_all"][0:i*numFrames,0]
            y_from0 = data[id_quad]["pos_all"][0:i*numFrames,1]
            z_from0 = data[id_quad]["pos_all"][0:i*numFrames,2]

            dxm = data[id_quad]["params"]["dxm"]
            dym = data[id_quad]["params"]["dym"]
            dzm = data[id_quad]["params"]["dzm"]

            quat = data[id_quad]["quat_all"][i*numFrames]

            if (config.orient == "NED"):
                z = -z
                z_from0 = -z_from0
                quat = np.array([quat[0], -quat[1], -quat[2], quat[3]])

            R = utils.quat2Dcm(quat)
            motorPoints = np.array([[dxm, -dym, dzm], [0, 0, 0], [dxm, dym, dzm], [-dxm, dym, dzm], [0, 0, 0], [-dxm, -dym, dzm]])
            motorPoints = np.dot(R, np.transpose(motorPoints))
            motorPoints[0,:] += x
            motorPoints[1,:] += y
            motorPoints[2,:] += z


            lines1[j].set_data(motorPoints[0,0:3], motorPoints[1,0:3])
            lines1[j].set_3d_properties(motorPoints[2,0:3])
            lines2[j].set_data(motorPoints[0,3:6], motorPoints[1,3:6])
            lines2[j].set_3d_properties(motorPoints[2,3:6])
            lines3[j].set_data(x_from0, y_from0)
            lines3[j].set_3d_properties(z_from0)



        titleTime.set_text(u"Time = {:.2f} s".format(time))

        return lines1, lines2


    def ini_plot():
        for i in range(len(data)):
            lines1[i].set_data(np.empty([1]), np.empty([1]))
            lines1[i].set_3d_properties(np.empty([1]))
            lines2[i].set_data(np.empty([1]), np.empty([1]))
            lines2[i].set_3d_properties(np.empty([1]))
            lines3[i].set_data(np.empty([1]), np.empty([1]))
            lines3[i].set_3d_properties(np.empty([1]))

        return lines1, lines2, lines3


    # Creating the Animation object
    line_ani = animation.FuncAnimation(fig, updateLines, init_func=ini_plot, frames=len(t_all[0:-2:numFrames]), interval=(Ts*1000*numFrames), blit=False)

    plt.show()
    if (ifsave):
        line_ani.save('Gifs/Raw/lines.gif', dpi=80, writer='imagemagick', fps=25)
        #Writer = animation.writers['ffmpeg']
        #writer = Writer(fps=15, metadata=dict(artist='Me'), bitrate=1800)
        #line_ani.save('/Gifs/lines.mp4', writer=writer)

    #plt.show()
    return line_ani
