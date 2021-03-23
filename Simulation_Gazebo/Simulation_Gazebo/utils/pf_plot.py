# -*- coding: utf-8 -*-
"""
author: John Bass
email: john.bobzwik@gmail.com
license: MIT
Please feel free to use and modify this, but keep the above information. Thanks!
"""

import numpy as np
from numpy import pi
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib import animation

import utils
import config

numFrames = 8

def pfPlot(t_all, data, ax, fig, Ts, color='blue', ifsave=False):



    line1, = ax.plot([], [], [], lw=2, color=color)
    line2, = ax.plot([], [], [], lw=2, color=color)
    line3, = ax.plot([], [], [], '--', lw=1, color=color)


    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Altitude')

    titleTime = ax.text2D(0.05, 0.95, "", transform=ax.transAxes)


    def updateLines(i):

        data_i = data[i*numFrames]
        time = t_all[i*numFrames]
        z,y,x = data_i.nonzero()
        cube = ax.scatter(z, y, x, zdir='z', c=data_i[z,y,x], cmap=plt.cm.rainbow, marker=',')
        #cbar = fig.colorbar(cube, shrink=0.6, aspect=5)
        titleTime.set_text(u"Time = {:.2f} s".format(time))

        return ax


    def ini_plot():

        line1.set_data(np.empty([1]), np.empty([1]))
        line1.set_3d_properties(np.empty([1]))
        line2.set_data(np.empty([1]), np.empty([1]))
        line2.set_3d_properties(np.empty([1]))
        line3.set_data(np.empty([1]), np.empty([1]))
        line3.set_3d_properties(np.empty([1]))

        return ax


    # Creating the Animation object
    line_ani = animation.FuncAnimation(fig, updateLines, init_func=ini_plot, frames=len(t_all[0:-2:numFrames]), interval=(Ts*1000*numFrames), blit=False)

    if (ifsave):
        line_ani.save('Gifs/Raw/animation_.gif', dpi=80, writer='imagemagick', fps=25)

    #plt.show()
    return line_ani