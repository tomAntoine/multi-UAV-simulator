# Multi-UAV-simulator
GNC of a SWARM of UAVs in a SITL by Tom Antoine and Alex Martinez - Cranfield University AVDC MSc 2021

This project is composed of two independant but similar repositories: 
- Simulation_Python : it is used to do numerical simulations on the guidance algorithm fully on Python (see below)
- Simulation_Gazebo : here, the python code for guidance is used as SITL, for a Gazebo/ROS/Ardupilot world (https://github.com/alexMarFar/multi-UAV-simulator.git)

# Simulation_Python

## Prerequisites:
Python 3.8.5 was employed, so full compability with that or after version
Basic Python libraries such as matplotlib, math, etc

## Main Files

### run_3D_simulation.py
It is the main script from where the simulation is launched. 

In the main, the scenario is selected and the quadcopters are initialized. 

Then the simulation is run by calling run_sim and the data can be visualized in plots or 3D animations.

### scenarios.py
It holds the main scenarios that were employed for V&V in the numerical simulation.

It also includes the explanation for each input needed to run the code.

### pf3d.py
It is the script for the Guidance algorithm, in this case the potential field in 3D.

It can be easily exchanged by another algorithm. It returns discrete waypoints.

### trajectory.py, waypoints.py and ctrl.py
They hold the control and trajectory generation based on waypoints.

### run_3D_simulation_loop.py
Created in order to test simulations many times with randomized parameters, in order to do statistical V&V. 
It is very similar to the run_3D_simulation, but doesnt plot results for each simulation, only histograms 
plotting relevant data from the many simulations done. 

### quadFiles folder

#### quad.py
In order to implement the drones, both agents and enemies, the Quad class was created. This is a file that 
is used to firstly initialize the drones as independent objects and to assign and update their attributes. 

This method of implementation allowed a very easy and intuitive use of the code, as well as an unlimited 
scalability of the SWARM.  

For the initialization to happen several inputs need to be provided. The main ones are the drone 
identification number “quad_id”, the keyword “mode”, the target identification number “id_targ”, the goal 
position “pos_goal” and the position of static obstacles “pos_obs”. 

#### initQuad.py
Initializes quad parameters
 

### utils folder
#### animation.py
Creates the 3D animation

#### display.py
Creates the plots for the analysis

## How to launch the simulation

1. Open Terminal 1
              
       cd Simulation_Python/
       python run_3D_simulation.py

2. or

        python run_3D_simulation_loop.py
