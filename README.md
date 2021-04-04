# Alex, please remove this section

for the gazebo file, you have the latest version so you can take yours, and just add the few functions from my folder simu_gazeb_for_integration in the quad file. like take off, recharge battery etc

otherwise for the simu_python, since i think i have the latest version, it is maybe better that you modify the folder i sent you. you can add your part of the report here. For the functions descriptions, directly as commentary, and for more general stuff, here.

for ex for un function, the norm is to do it like this:

class quad():
	"""
	here you add blabla if you want for the class
	"""

	def function1():
	""" 
	here blabla for function1
	"""
	return 1


# GDP

This folder is composed of two independant but similar repositeries: 
- Simulation_Python
- Simulation_Gazebo

# Simulation_python 

The file named run_3D_simulation_loop was created in order to test simulations many times with randomized parameters, in order to do statistical V&V. It is very similar to the run_3D_simulation, but doesnt plot results for each simulation, only histograms plotting relevant data from the many simulations done. 

