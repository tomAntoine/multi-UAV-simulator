# Alex, please remove this section

for the gazebo file, you have the latest version so you can take yours, and just add the few functions from my folder simu_gazeb_for_integration in the quad file. like take off, recharge battery etc


here they are:

    if self.mode == 'home':
        ChangeMode(self.vehicle,"RTL")
        # be able to nkow when RTL is done 
        if not vehicle.armed :
            print("back home")
            
    if self.mode == 'takeoff':
        arm_and_takeoff(self.vehicle,3)
        print("takeoff finished")
        
    if self.mode == "charging":
        self.vehicle.battery.level += 10
        if self.vehicle.battery.level == 100:
            print('charged')
            
    if self.mode == 'land':
        pos_dyn_obs = [x[1] for x in self.pos_quads if  (x[0] != self.quad_id)]
        try :
            temp_pos_obs = np.vstack((self.pos_obs, pos_dyn_obs)).astype(float)
        except :
            temp_pos_obs = np.vstack((self.pos_obs)).astype(float)
        self.pos_goal = np.hstack([self.pos[0], self.pos[1], -0.5]).astype(float)


otherwise for the simu_python, since i think i have the latest version, it is maybe better that you modify the folder i sent you. you can add your part of the report here. For the functions descriptions, directly as commentary, and for more general stuff, here.

for ex for un function, the norm is to do it like this:

	class quad :
		"""
		here you add blabla if you want for the class
		"""

		def function1():
			""" 
			here blabla for function1
			"""
			return 1

BTW, i added a new file called scenarios where there are all defined,
i think it is better and looks better than defining them in the main function, it didn't made any sens. 



# GDP

This folder is composed of two independant but similar repositeries: 
- Simulation_Python : it is used to do numerical simulations fully on python
- Simulation_Gazebo : here, the python code for guidance is used as SITL, for a Gazebo/ROS world

# Simulation_python 

The file named run_3D_simulation_loop was created in order to test simulations many times with randomized parameters, in order to do statistical V&V. It is very similar to the run_3D_simulation, but doesnt plot results for each simulation, only histograms plotting relevant data from the many simulations done. 


