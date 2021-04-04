
from collections import deque
import numpy as np
import matplotlib.pyplot as plt
import math
from mpl_toolkits.mplot3d import Axes3D

# Parameters
KP = 12.0  # attractive potential gain
ETA = 60.0  # repulsive potential gain
VOLUME_WIDTH = 20.0 # potential volume width [m]
VOLUME_WIDTHz = 5 # potential volume width [m]
#AREA_WIDTH = 30.0  # potential area width [m]
# the number of previous positions used to check oscillations
OSCILLATIONS_DETECTION_LENGTH = 3

show_animation = False


def calc_potential_field(gx, gy, gz, ox, oy, oz, reso, rr, sx, sy, sz):
    minx = min(min(ox), sx, gx) - VOLUME_WIDTH / 2.0
    miny = min(min(oy), sy, gy) - VOLUME_WIDTH / 2.0
    minz = min(min(oz), sz, gz) - VOLUME_WIDTHz / 2.0

    maxx = max(max(ox), sx, gx) + VOLUME_WIDTH / 2.0
    maxy = max(max(oy), sy, gy) + VOLUME_WIDTH / 2.0
    maxz = max(max(oz), sz, gz) + VOLUME_WIDTHz / 2.0

    xw = int(round((maxx - minx) / reso))
    yw = int(round((maxy - miny) / reso))
    zw = int(round((maxz - minz) / reso))

    # calc each potential
    pmap = np.array([[[0.0 for k in range(zw)] for j in range(yw)] for i in range(xw)])

    for ix in range(xw):
        x = ix * reso + minx

        for iy in range(yw):
            y = iy * reso + miny

            for iz in range(zw):
                z = iz * reso + minz

                ug = calc_attractive_potential(x, y, z, gx, gy, gz)
                uo = calc_repulsive_potential(x, y, z, ox, oy, oz, rr)
                uf = ug + uo
                pmap[ix,iy,iz] = uf

    # print(pmap)
    # print(pmap.shape)
    # draw_heatmap(pmap[int(VOLUME_WIDTH/2+1)], ax1)
    # draw_heatmap(pmap[:,int(VOLUME_WIDTH/2+1)],ax2)
    # draw_heatmap(pmap[int(VOLUME_WIDTH/2+2)],ax3)
    # draw_heatmap(pmap[int(VOLUME_WIDTH/2+4)],ax4)

    data = pmap[int(VOLUME_WIDTH/2):len(pmap)-int(VOLUME_WIDTH/2),
            int(VOLUME_WIDTH/2):len(pmap[0])-int(VOLUME_WIDTH/2),
            int(VOLUME_WIDTHz/2):len(pmap[0,0])-int(VOLUME_WIDTHz/2)]


    return pmap, minx, miny, minz, data


import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import figure
from ipywidgets import interact



def calc_attractive_potential(x, y, z, gx, gy, gz):
    res = 0.5 * KP * np.sqrt((x - gx)**2 + (y - gy)**2 + (z - gz)**2)
    return res


def calc_repulsive_potential(x, y, z, ox, oy, oz, rr):
    # search nearest obstacle
    minid = -1
    dmin = float("inf")
    for i, _ in enumerate(ox):
        d = np.sqrt((x - ox[i])**2 + (y - oy[i])**2 + (z - oz[i])**2)
        if dmin >= d:
            dmin = d
            minid = i

    # calc repulsive potential
    dq = np.sqrt((x - ox[minid])**2 + (y - oy[minid])**2 + (z - oz[minid])**2)

    if dq <= rr:
        if dq <= 0.1:
            dq = 0.1

        return 0.5 * ETA * (1.0 / dq - 1.0 / rr) ** 2
    else:
        return 0.0


def get_motion_model():
    # dx, dy, dz
    a = 1 #np.sqrt(3)/3
    b = 1 #np.sqrt(2)/2
    motion = [[1, 0, 0],
            [-1, 0, 0],
            [0, 1, 0],
            [0, -1, 0],
            [0, 0, 1],
            [0, 0, -1],
            [a, a, a],
            [-a, a, a],
            [-a, -a, a],
            [-a, -a, -a],
            [a, -a, -a],
            [a, a, -a],
            [-a, a, -a],
            [a, -a, a],
            [b, b, 0],
            [-b, b, 0],
            [b, -b, 0],
            [-b, -b, 0],
            [0, b, b],
            [0, -b, b],
            [0, b, -b],
            [0, -b, -b],
            [b, 0, b],
            [-b, 0, b],
            [b, 0, -b],
            [-b, 0, -b]]

    return motion

def oscillations_detection(previous_ids, ix, iy, iz):
    previous_ids.append((ix, iy, iz))

    if (len(previous_ids) > OSCILLATIONS_DETECTION_LENGTH):
        previous_ids.popleft()

    # check if contains any duplicates by copying into a set
    previous_ids_set = set()
    for index in previous_ids:
        if index in previous_ids_set:
            return True
        else:
            previous_ids_set.add(index)
    return False


def potential_field_planning(sx, sy, sz, gx, gy, gz, ox, oy, oz, reso, rr):

    # calc potential field
    pmap, minx, miny, minz, data = calc_potential_field(gx, gy, gz, ox, oy, oz, reso, rr, sx, sy, sz)

    # search path
    d = np.sqrt((sx - gx)**2 + (sy - gy)**2 + (sz - gz)**2)

    ix = round((sx - minx) / reso)
    iy = round((sy - miny) / reso)
    iz = round((sz - minz) / reso)

    gix = round((gx - minx) / reso)
    giy = round((gy - miny) / reso)
    giz = round((gz - minz) / reso)

    rx, ry, rz = [sx], [sy], [sz]
    motion = get_motion_model()
    previous_ids = deque()

    count = 0

    while d >= reso:
        minp = float("inf")
        minix, miniy, minz = -1, -1, -1
        for i in range(len(motion)):
            inx = int(ix + motion[i][0])
            iny = int(iy + motion[i][1])
            inz = int(iz + motion[i][2])
            if inx >= len(pmap) or iny >= len(pmap[0]) or inz >= len(pmap[0][0]) or inx < 0 or iny < 0 or inz < 0:
                p = float("inf")  # outside area
                print("outside potential!")
            else:
                p = pmap[inx][iny][inz]
            if minp > p:
                minp = p
                minix = inx
                miniy = iny
                miniz = inz
        ix = minix
        iy = miniy
        iz = miniz

        xp = ix * reso + minx
        yp = iy * reso + miny
        zp = iz * reso + minz

        d = np.sqrt((gx - xp)**2 + (gy - yp)**2 + (gz - zp)**2)


        rx.append(xp)
        ry.append(yp)
        rz.append(zp)

        count += 1

        if count > 100:
            #print("too many runs")
            break



    #print("Goal!!")

    return rx, ry, rz, data


def draw_heatmap(data,ax):
    data = np.array(data).T
    ax.pcolor(data, vmax=100.0, cmap=plt.cm.Blues)



def pf3d(pos_ini, pos_goal, pos_obs):
    #print("potential_field_planning start")
    grid_size=1
    robot_radius=4.5


    sx = pos_ini[0]  # start x position [m]
    sy = pos_ini[1]  # start y positon [m]
    sz = -pos_ini[2]  # start z positon [m]

    gx = pos_goal[0] # goal x position [m]
    gy = pos_goal[1] # goal y position [m]
    gz = -pos_goal[2] # goal z position [m]


    ox = pos_obs[:,0]  # obstacle x position list [m]
    oy = pos_obs[:,1]  # obstacle y position list [m]
    oz = -pos_obs[:,2]  # obstacle y position list [m]

    # path generation
    rx, ry, rz, data = potential_field_planning(sx, sy, sz, gx, gy, gz, ox, oy, oz, grid_size, robot_radius)
    rz = -1*np.array(rz)
    wps_pf3d = [rx, ry, rz]
    wps_pf3d = np.transpose(wps_pf3d)
    wps_pf3d = np.array(wps_pf3d).reshape(-1,3)


    return wps_pf3d, data
    #fig = plt.figure()
    #ax = fig.add_subplot(111, projection='3d')
    #ax.plot(rx, ry, rz)
    #ax.plot(ox, oy, oz, 'ro')
    #ax.plot(sx, sy, sz, 'bo')
    #ax.plot(gx, gy, gz, 'go')
    #plt.show()

def main():
    #print("potential_field_planning start")

    pos_ini = [0,0,0]
    pos_goal = [3,3,-10]
    pos_obs = np.array([[1,1,-1], [2,2,-2], [1,5,-3]])
    grid_size=1
    robot_radius=4.5


    sx = pos_ini[0]  # start x position [m]
    sy = pos_ini[1]  # start y positon [m]
    sz = -pos_ini[2]  # start z positon [m]

    gx = pos_goal[0] # goal x position [m]
    gy = pos_goal[1] # goal y position [m]
    gz = -pos_goal[2] # goal z position [m]


    ox = pos_obs[:,0]  # obstacle x position list [m]
    oy = pos_obs[:,1]  # obstacle y position list [m]
    oz = -pos_obs[:,2]  # obstacle y position list [m]

    # path generation
    rx, ry, rz, data = potential_field_planning(sx, sy, sz, gx, gy, gz, ox, oy, oz, grid_size, robot_radius)
    rz = -1*np.array(rz)
    wps_pf3d = [rx, ry, rz]
    wps_pf3d = np.transpose(wps_pf3d)
    wps_pf3d = np.array(wps_pf3d).reshape(-1,3)


    plt.show()


if __name__ == '__main__':

    print(__file__ + " start!!")
    main()
    print(__file__ + " Done!!")


