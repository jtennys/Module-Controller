#!/usr/bin/env python
import roslib; roslib.load_manifest('module_controller')
import rospy
import serial
from ModuleControllerGUI import *

if __name__ == "__main__":
	# Create arrays for module data...
	# Array that stores current module angle as a 10 bit unsigned int
	# in the max range of motion for the module.
	moduleIntAngles = []
	# Array that stores the current module angle as a float in degrees.
	moduleFloatAngles = []
	# Array that stores the offset of where angle 0 is for each module.
	moduleOffsets = []

	# Open the Serial Port.


	# Get the number of modules in this robot.
	numModules = 3

	# Create the graphical user interface.
	modGUI = ModuleControllerGUI(numModules)
	modGUI.start()

	gtk.main()
	
	# Populate the arrays with one extra value, so that our
	# modules can be stored in the index range 1 to numModules.
	for i in range(0,numModules+1):
		moduleIntAngles.append(int(0))
		moduleFloatAngles.append(float(0.0))
		moduleOffsets.append(int(0))

	# This is where we would grab the offsets.
	# For now, we fake it.
	for i in range(1,numModules+1):
		moduleOffsets[i] = 511

	# While there is no user input, update angles and x, y, z position.
	# Just forced to loop forever at the moment.
#	while True:
#		for i in range(1,numModules+1):
#			# Grab the angle from module i.
#			# For now, we fake it.
#			moduleIntAngles[i] = 600+(30*i)
#			moduleFloatAngles[i] = ((moduleIntAngles[i] - moduleOffsets[i])/1023.0)*300.0
#			interface.textBoxList[i].set_text(str(moduleFloatAngles[i]))
#			print moduleFloatAngles[i]
