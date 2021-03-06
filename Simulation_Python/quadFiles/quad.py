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
from numpy import sin, cos, tan, pi, sign
from scipy.integrate import ode

from quadFiles.initQuad import sys_params, init_cmd, init_state
import utils
import config

from trajectory import Trajectory
from ctrl import Control

deg2rad = pi/180.0



class Quadcopter:
    
    '''
    In order to implement the drones, both agents and enemies, the Quad class was created. 
    This is a file that is used to firstly initialize the drones as independent objects and 
    to assign and update their attributes. 

    This method of implementation allowed a very easy and intuitive use of the code, as well
    as an unlimited scalability of the SWARM.  

    For the initialization to happen several inputs need to be provided. The main ones are the 
    drone identification number “quad_id”, the keyword “mode”, the target identification number 
    “id_targ”, the goal position “pos_goal” and the position of static obstacles “pos_obs”. 

    '''

    def __init__(self, Ti, Tf, ctrlType, trajSelect, numTimeStep, Ts, quad_id = None, mode = 'ennemy', id_targ = -1, color='blue', pos_ini = [0,0,0], pos_goal= [1,1,-1], pos_obs = None):
        '''
        It is used to store in the basic attributes the inputs already mentioned in the previous
        sections. 
        
        In the case of the one employed by Python simulation environment, the command for an initial
        stable hover and the initial state of the drone are initialized. For that, the dynamics, 
        kinetics, kinematics and control are also loaded. Finally, the storage of drone data is 
        pre-allocated.

        '''
        # Quad Params
        # ---------------------------

        self.quad_id = quad_id


        self.pos_ini = pos_ini
        self.pos_goal = pos_goal
        self.pos_goal_ini = pos_goal
        self.pos_obs = pos_obs

        self.id_targ = id_targ

        self.pos_quads = None

        self.mode = mode
        self.mode_ini = mode
        self.neutralize = False
        self.ctrlType = ctrlType
        self.trajSelect = trajSelect

        self.params = sys_params()

        self.t_update = Ti
        self.t_track = 0
        self.Ti = Ti
        self.Ts = Ts
        self.Tf = Tf

        # Command for initial stable hover
        # ---------------------------
        ini_hover = init_cmd(self.params)
        self.params["FF"] = ini_hover[0]         # Feed-Forward Command for Hover
        self.params["w_hover"] = ini_hover[1]    # Motor Speed for Hover
        self.params["thr_hover"] = ini_hover[2]  # Motor Thrust for Hover
        self.thr = np.ones(4)*ini_hover[2]
        self.tor = np.ones(4)*ini_hover[3]

        # Initial State
        # ---------------------------

        self.state = init_state(self.params,pos_ini)

        self.pos   = self.state[0:3]
        self.quat  = self.state[3:7]
        self.vel   = self.state[7:10]
        self.omega = self.state[10:13]
        self.wMotor = np.array([self.state[13], self.state[15], self.state[17], self.state[19]])
        self.vel_dot = np.zeros(3)
        self.omega_dot = np.zeros(3)
        self.acc = np.zeros(3)

        self.previous_pos_targ = self.pos
        self.previous_pos_us = self.pos

        self.extended_state()
        self.forces()

        # Set Integrator
        # ---------------------------
        self.integrator = ode(self.state_dot).set_integrator('dopri5', first_step='0.00005', atol='10e-6', rtol='10e-6')
        self.integrator.set_initial_value(self.state, Ti)

        self.traj = Trajectory(self.psi, ctrlType, trajSelect, pos_ini=self.pos, pos_goal=pos_goal, pos_obs = pos_obs, Tf = self.Tf, t_ini=0)
        self.ctrl = Control(self.params["w_hover"], self.traj.yawType)

        self.t_all          = np.zeros(numTimeStep)
        self.s_all          = np.zeros([numTimeStep, len(self.state)])
        self.pos_all        = np.zeros([numTimeStep, len(self.pos)])
        self.vel_all        = np.zeros([numTimeStep, len(self.vel)])
        self.quat_all       = np.zeros([numTimeStep, len(self.quat)])
        self.omega_all      = np.zeros([numTimeStep, len(self.omega)])
        self.euler_all      = np.zeros([numTimeStep, len(self.euler)])
        self.sDes_traj_all  = np.zeros([numTimeStep, len(self.traj.sDes)])
        self.sDes_calc_all  = np.zeros([numTimeStep, len(self.ctrl.sDesCalc)])
        self.w_cmd_all      = np.zeros([numTimeStep, len(self.ctrl.w_cmd)])
        self.wMotor_all     = np.zeros([numTimeStep, len(self.wMotor)])
        self.thr_all        = np.zeros([numTimeStep, len(self.thr)])
        self.tor_all        = np.zeros([numTimeStep, len(self.tor)])

        self.color = color

        self.pf_map_all = np.empty(numTimeStep,dtype=list)

    def extended_state(self):
        '''
        Only for the python environment. It computes the rotation matrix of the current state 
        and its Euler angles too.
        '''

        # Rotation Matrix of current state (Direct Cosine Matrix)
        self.dcm = utils.quat2Dcm(self.quat)

        # Euler angles of current state
        YPR = utils.quatToYPR_ZYX(self.quat)
        self.euler = YPR[::-1] # flip YPR so that euler state = phi, theta, psi
        self.psi   = YPR[0]
        self.theta = YPR[1]
        self.phi   = YPR[2]


    def forces(self):
        '''
        Only for the python environment. It computes the rotor thrusts and torques.
        '''

        # Rotor thrusts and torques
        self.thr = self.params["kTh"]*self.wMotor*self.wMotor
        self.tor = self.params["kTo"]*self.wMotor*self.wMotor

    def state_dot(self, t, state, cmd, wind):
        '''
        Only for the python environment. It calls the dynamic parameters of each drones in
        terms of mass, inertia and damping. It also calls the aerodynamic and propulsion 
        coefficients such as drag, thrust or torque. It imports the state vector of each drone,
        in terms of position, orientation, velocity or angular rate. After that, it studies the 
        motor dynamics and rotor forces. The wind model is also called here. Finally, it solves 
        the state derivatives.
        '''

        # Import Params
        # ---------------------------
        mB   = self.params["mB"]
        g    = self.params["g"]
        dxm  = self.params["dxm"]
        dym  = self.params["dym"]
        IB   = self.params["IB"]
        IBxx = IB[0,0]
        IByy = IB[1,1]
        IBzz = IB[2,2]
        Cd   = self.params["Cd"]

        kTh  = self.params["kTh"]
        kTo  = self.params["kTo"]
        tau  = self.params["tau"]
        kp   = self.params["kp"]
        damp = self.params["damp"]
        minWmotor = self.params["minWmotor"]
        maxWmotor = self.params["maxWmotor"]

        IRzz = self.params["IRzz"]
        if (config.usePrecession):
            uP = 1
        else:
            uP = 0

        # Import State Vector
        # ---------------------------
        x      = state[0]
        y      = state[1]
        z      = state[2]
        q0     = state[3]
        q1     = state[4]
        q2     = state[5]
        q3     = state[6]
        xdot   = state[7]
        ydot   = state[8]
        zdot   = state[9]
        p      = state[10]
        q      = state[11]
        r      = state[12]
        wM1    = state[13]
        wdotM1 = state[14]
        wM2    = state[15]
        wdotM2 = state[16]
        wM3    = state[17]
        wdotM3 = state[18]
        wM4    = state[19]
        wdotM4 = state[20]

        # Motor Dynamics and Rotor forces (Second Order System: https://apmonitor.com/pdc/index.php/Main/SecondOrderSystems)
        # ---------------------------

        uMotor = cmd
        wddotM1 = (-2.0*damp*tau*wdotM1 - wM1 + kp*uMotor[0])/(tau**2)
        wddotM2 = (-2.0*damp*tau*wdotM2 - wM2 + kp*uMotor[1])/(tau**2)
        wddotM3 = (-2.0*damp*tau*wdotM3 - wM3 + kp*uMotor[2])/(tau**2)
        wddotM4 = (-2.0*damp*tau*wdotM4 - wM4 + kp*uMotor[3])/(tau**2)

        wMotor = np.array([wM1, wM2, wM3, wM4])
        wMotor = np.clip(wMotor, minWmotor, maxWmotor)
        thrust = kTh*wMotor*wMotor
        torque = kTo*wMotor*wMotor

        ThrM1 = thrust[0]
        ThrM2 = thrust[1]
        ThrM3 = thrust[2]
        ThrM4 = thrust[3]
        TorM1 = torque[0]
        TorM2 = torque[1]
        TorM3 = torque[2]
        TorM4 = torque[3]

        # Wind Model
        # ---------------------------
        [velW, qW1, qW2] = wind.randomWind(t)
        # velW = 0

        # velW = 5          # m/s
        # qW1 = 0*deg2rad    # Wind heading
        # qW2 = 60*deg2rad     # Wind elevation (positive = upwards wind in NED, positive = downwards wind in ENU)

        # State Derivatives (from PyDy) This is already the analytically solved vector of MM*x = RHS
        # ---------------------------
        if (config.orient == "NED"):
            DynamicsDot = np.array([
                [                                                                                                                                   xdot],
                [                                                                                                                                   ydot],
                [                                                                                                                                   zdot],
                [                                                                                                        -0.5*p*q1 - 0.5*q*q2 - 0.5*q3*r],
                [                                                                                                         0.5*p*q0 - 0.5*q*q3 + 0.5*q2*r],
                [                                                                                                         0.5*p*q3 + 0.5*q*q0 - 0.5*q1*r],
                [                                                                                                        -0.5*p*q2 + 0.5*q*q1 + 0.5*q0*r],
                [     (Cd*sign(velW*cos(qW1)*cos(qW2) - xdot)*(velW*cos(qW1)*cos(qW2) - xdot)**2 - 2*(q0*q2 + q1*q3)*(ThrM1 + ThrM2 + ThrM3 + ThrM4))/mB],
                [     (Cd*sign(velW*sin(qW1)*cos(qW2) - ydot)*(velW*sin(qW1)*cos(qW2) - ydot)**2 + 2*(q0*q1 - q2*q3)*(ThrM1 + ThrM2 + ThrM3 + ThrM4))/mB],
                [ (-Cd*sign(velW*sin(qW2) + zdot)*(velW*sin(qW2) + zdot)**2 - (ThrM1 + ThrM2 + ThrM3 + ThrM4)*(q0**2 - q1**2 - q2**2 + q3**2) + g*mB)/mB],
                [                                    ((IByy - IBzz)*q*r - uP*IRzz*(wM1 - wM2 + wM3 - wM4)*q + ( ThrM1 - ThrM2 - ThrM3 + ThrM4)*dym)/IBxx], # uP activates or deactivates the use of gyroscopic precession.
                [                                    ((IBzz - IBxx)*p*r + uP*IRzz*(wM1 - wM2 + wM3 - wM4)*p + ( ThrM1 + ThrM2 - ThrM3 - ThrM4)*dxm)/IByy], # Set uP to False if rotor inertia is not known (gyro precession has negigeable effect on drone dynamics)
                [                                                                               ((IBxx - IByy)*p*q - TorM1 + TorM2 - TorM3 + TorM4)/IBzz]])
        elif (config.orient == "ENU"):
            DynamicsDot = np.array([
                [                                                                                                                                   xdot],
                [                                                                                                                                   ydot],
                [                                                                                                                                   zdot],
                [                                                                                                        -0.5*p*q1 - 0.5*q*q2 - 0.5*q3*r],
                [                                                                                                         0.5*p*q0 - 0.5*q*q3 + 0.5*q2*r],
                [                                                                                                         0.5*p*q3 + 0.5*q*q0 - 0.5*q1*r],
                [                                                                                                        -0.5*p*q2 + 0.5*q*q1 + 0.5*q0*r],
                [     (Cd*sign(velW*cos(qW1)*cos(qW2) - xdot)*(velW*cos(qW1)*cos(qW2) - xdot)**2 + 2*(q0*q2 + q1*q3)*(ThrM1 + ThrM2 + ThrM3 + ThrM4))/mB],
                [     (Cd*sign(velW*sin(qW1)*cos(qW2) - ydot)*(velW*sin(qW1)*cos(qW2) - ydot)**2 - 2*(q0*q1 - q2*q3)*(ThrM1 + ThrM2 + ThrM3 + ThrM4))/mB],
                [ (-Cd*sign(velW*sin(qW2) + zdot)*(velW*sin(qW2) + zdot)**2 + (ThrM1 + ThrM2 + ThrM3 + ThrM4)*(q0**2 - q1**2 - q2**2 + q3**2) - g*mB)/mB],
                [                                    ((IByy - IBzz)*q*r + uP*IRzz*(wM1 - wM2 + wM3 - wM4)*q + ( ThrM1 - ThrM2 - ThrM3 + ThrM4)*dym)/IBxx], # uP activates or deactivates the use of gyroscopic precession.
                [                                    ((IBzz - IBxx)*p*r - uP*IRzz*(wM1 - wM2 + wM3 - wM4)*p + (-ThrM1 - ThrM2 + ThrM3 + ThrM4)*dxm)/IByy], # Set uP to False if rotor inertia is not known (gyro precession has negigeable effect on drone dynamics)
                [                                                                               ((IBxx - IBzz)*p*q + TorM1 - TorM2 + TorM3 - TorM4)/IBzz]])


        # State Derivative Vector
        # ---------------------------
        sdot     = np.zeros([21])
        sdot[0]  = DynamicsDot[0]
        sdot[1]  = DynamicsDot[1]
        sdot[2]  = DynamicsDot[2]
        sdot[3]  = DynamicsDot[3]
        sdot[4]  = DynamicsDot[4]
        sdot[5]  = DynamicsDot[5]
        sdot[6]  = DynamicsDot[6]
        sdot[7]  = DynamicsDot[7]
        sdot[8]  = DynamicsDot[8]
        sdot[9]  = DynamicsDot[9]
        sdot[10] = DynamicsDot[10]
        sdot[11] = DynamicsDot[11]
        sdot[12] = DynamicsDot[12]
        sdot[13] = wdotM1
        sdot[14] = wddotM1
        sdot[15] = wdotM2
        sdot[16] = wddotM2
        sdot[17] = wdotM3
        sdot[18] = wddotM3
        sdot[19] = wdotM4
        sdot[20] = wddotM4

        self.acc = sdot[7:10]

        return sdot

    def print_mode(self, mode_prev, mode_new):
        '''
        Only for the python environment. It updates the mode of the drone by printing the 
        identification number of the drone, its previous mode and its current one.
        '''

        print("Drone {0} switch from {1} to {2}.".format(self.quad_id, mode_prev, mode_new))

    def update(self, t, Ts, wind, i, quad):
        '''
        It is used to update the attributes of each drone. First of all, the Boolean must 
        be changed into true for specific modes and to false for others following their nature. 

        If the initial mode is “enemy”, no update is required as neither collision avoidance
        or tracking features are activated. However, in the python environment, if the target 
        becomes neutralized and its mode is changed into “fall”, the goal position will be 
        updated once as its initial goal position with null height to simulate its crash. This 
        is done by the MAVlink command “stabilize” in the more complex environment.

        If the initial mode is not “enemy” and the Boolean is true, then update occurs. In the 
        case of an agent in “guided” mode, it will update the position of other drones as dynamic 
        obstacles to enhance collision avoidance. For that, it will compare the identification 
        number of each drone and verify it does not match its own. Then, the temporal position of 
        obstacles will be jointly stored containing the static and dynamic ones. After that, the 
        guidance algorithm will be called and the trajectory updated. For an automated transition 
        of modes, a distance threshold can be employed to consider the goal position reached. For 
        that, the norm of the difference between the current position and the goal one is computed 
        and compared with said threshold. If it is smaller, the mode can be switched to “home”.

        In the case of an agent in “track” mode, it will update both the position of other drones 
        for collision avoidance and its goal position. For that, the collision avoidance happens 
        similarly to “guided” mode although in this case, it will discard the identification numbers 
        that match both its own and the target it is tasked to follow. As for the tracking aspect, 
        the target position is assumed to be provided by the Situation Awareness team. However, in 
        the initial stages it is extracted from the environment. For a basic tracking, the goal 
        position is assumed to be the target position with an additional half meter in z axis, in 
        order for the agent to hover over it. Nonetheless, this means that the agent will never be 
        able to hover exactly over the target. For an improved tracking performance, the future 
        position of the target is estimated using a constant velocity assumption. This discretized 
        velocity is computed based on its current and past positions. This estimation can be further 
        improved by including the continuous velocity, acceleration or by tuning its parameters. 
        This estimation, after adding the hovering distance is employed for the path planning. In 
        order to check whether or not a target has been neutralized two thresholds are needed, one 
        associated with time and another one with the distance between the tracking agent and its 
        target. In this project, the thresholds are set to 1m for 1.5s (three times the update time).
        If both are met, the Boolean neutralize becomes true. This one will later be used to change 
        the target mode from “enemy” to “neutralized”. Once again, a normal automated transition for 
        the agent would be “home” mode. 
        
        In the case of the remaining modes associated with actions, the longer ones can also include 
        collision avoidance as a feature and therefore require updating. This is the case of “home”, 
        “land” or “hover”, but not the case of “charging” or “takeoff”. Nonetheless, in the more 
        complex environment, most of these are predefined and can be directly sent as MAVlink commands.
        
        After the mode and goal position are updated, the trajectories are overwritten. For that, 
        the current states are updated and the commands recomputed.

        '''      

        if (self.t_update > self.Tf) or (t == 0):
            update = True
            self.t_update = 0
        else :
            update = False



        if self.mode == "fall":

            self.pos_goal = np.hstack([self.pos[0], self.pos[1], -0.5]).astype(float)
            self.traj = Trajectory(self.psi, self.ctrlType, self.trajSelect, pos_ini=self.pos, pos_goal=self.pos_goal, pos_obs = np.array([[0,0,0]]), Tf = self.Tf, t_ini = t)
            self.ctrl = Control(self.params["w_hover"], self.traj.yawType)
            self.mode = "neutralized"

            self.print_mode("ennemy", "fall")



# suggestion for traking algos:

        if (self.mode != 'ennemy' and update) :


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

                estimate_pos_targ = 3*pos_targ-2*self.previous_pos_targ
                self.pos_goal = np.hstack((estimate_pos_targ) + [0,0,1.5]).astype(float)

                dist = np.sqrt((self.previous_pos_us[0]-self.previous_pos_targ[0])**2+(self.previous_pos_us[1]-self.previous_pos_targ[1])**2+(self.previous_pos_us[2]-self.previous_pos_targ[2])**2)
                print(dist)
                if dist < 1:
                    self.t_track += Ts
                    if self.t_track > 3*Ts:
                        self.neutralize = True
                        self.mode = "home"
                        self.print_mode("track", "home")
                        self.t_track = t - Ts
                else :
                    self.t_track = 0
                self.previous_pos_targ = pos_targ
                self.previous_pos_us = self.pos

            if self.mode == 'guided':
                try :
                    pos_dyn_obs = [x[1] for x in self.pos_quads if  (x[0] != self.quad_id)]
                    temp_pos_obs = np.vstack((self.pos_obs, pos_dyn_obs)).astype(float)
                except :
                    temp_pos_obs = np.vstack((self.pos_obs)).astype(float)
                dist = np.sqrt((self.pos[0]-self.pos_goal[0])**2+(self.pos[1]-self.pos_goal[1])**2+(self.pos[2]-self.pos_goal[2])**2)
                if dist < 1:
                    self.mode = "home"
                    self.print_mode("guided", "home")

            if self.mode == 'home':
                pos_dyn_obs = [x[1] for x in self.pos_quads if  (x[0] != self.quad_id)]
                temp_pos_obs = np.vstack((self.pos_obs, pos_dyn_obs)).astype(float)
                self.pos_goal = np.hstack(self.pos_ini).astype(float)
                dist = np.sqrt((self.pos[0]-self.pos_goal[0])**2+(self.pos[1]-self.pos_goal[1])**2+(self.pos[2]-self.pos_goal[2])**2)
                if dist < 1:
                    self.mode = "land"
                    self.print_mode("home", "land")


            if self.mode == 'land':
                pos_dyn_obs = [x[1] for x in self.pos_quads if  (x[0] != self.quad_id)]
                temp_pos_obs = np.vstack((self.pos_obs, pos_dyn_obs)).astype(float)
                self.pos_goal = np.hstack([self.pos[0], self.pos[1], -0.5]).astype(float)

            if self.mode == 'hover':
                pos_dyn_obs = [x[1] for x in self.pos_quads if  (x[0] != self.quad_id)]
                temp_pos_obs = np.vstack((self.pos_obs, pos_dyn_obs)).astype(float)
                self.pos_goal = np.hstack(self.pos).astype(float)

            self.traj = Trajectory(self.psi, self.ctrlType, self.trajSelect, pos_ini=self.pos, pos_goal=self.pos_goal, pos_obs = temp_pos_obs, Tf = self.Tf, t_ini = t)
            self.ctrl = Control(self.params["w_hover"], self.traj.yawType)

        prev_vel   = self.vel
        prev_omega = self.omega

        self.sDes = self.traj.desiredState(t, Ts, self.pos)

        self.integrator.set_f_params(self.ctrl.w_cmd, wind)
        self.state = self.integrator.integrate(t, t+Ts)

        self.pos   = self.state[0:3]
        self.quat  = self.state[3:7]
        self.vel   = self.state[7:10]
        self.omega = self.state[10:13]
        self.wMotor = np.array([self.state[13], self.state[15], self.state[17], self.state[19]])

        self.vel_dot = (self.vel - prev_vel)/Ts
        self.omega_dot = (self.omega - prev_omega)/Ts

        self.extended_state()
        self.forces()

        self.t_all[i]             = t
        self.s_all[i,:]           = self.state
        self.pos_all[i,:]         = self.pos
        self.vel_all[i,:]         = self.vel
        self.quat_all[i,:]        = self.quat
        self.omega_all[i,:]       = self.omega
        self.euler_all[i,:]       = self.euler
        self.sDes_traj_all[i,:]   = self.traj.sDes
        self.sDes_calc_all[i,:]   = self.ctrl.sDesCalc
        self.w_cmd_all[i,:]       = self.ctrl.w_cmd
        self.wMotor_all[i,:]      = self.wMotor
        self.thr_all[i,:]         = self.thr
        self.tor_all[i,:]         = self.tor

        self.pf_map_all[i]      = self.traj.data

        # Trajectory for Desired States
        # ---------------------------
        self.sDes = self.traj.desiredState(t, Ts, self.pos)

        # Generate Commands (for next iteration)
        # ---------------------------
        self.ctrl.controller(self.traj, quad, self.sDes, Ts)

        self.t_update += Ts

    def init(self,Ts,quad):
        '''
        It is employed to input the first value of the attributes after initialization 
        in the python environment. 
        '''

        # Trajectory for First Desired States
        # ---------------------------
        self.sDes = self.traj.desiredState(0, Ts, self.pos)

        # Generate First Commands
        # ---------------------------
        self.ctrl.controller(self.traj, quad, self.sDes, Ts)

        self.t_all[0]            = self.Ti
        self.s_all[0,:]          = self.state
        self.pos_all[0,:]        = self.pos
        self.vel_all[0,:]        = self.vel
        self.quat_all[0,:]       = self.quat
        self.omega_all[0,:]      = self.omega
        self.euler_all[0,:]      = self.euler
        self.sDes_traj_all[0,:]  = self.traj.sDes
        self.sDes_calc_all[0,:]  = self.ctrl.sDesCalc
        self.w_cmd_all[0,:]      = self.ctrl.w_cmd
        self.wMotor_all[0,:]     = self.wMotor
        self.thr_all[0,:]        = self.thr
        self.tor_all[0,:]        = self.tor

        self.pf_map_all[0]      = self.traj.data
