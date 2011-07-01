#!/usr/bin/env python
import roslib; roslib.load_manifest('module_controller')
from module_controller.srv import *
import rospy
import serial
import threading
import time
import math

# A simple definition that converts degree values to radians.
def degToRad(degrees):
	return degrees*(math.pi/180)

# A simple definition that converts radian values to degrees.
def radToDeg(radians):
	return radians*(180/math.pi)

# The following pair of functions provide thread safety for our node, since
# it has a possibility for multiple threads to be using the serial port.
def serialCommEnter():
	# Pull in the global variable serialInUse for checking and modifying.
	global serialInUse

	# While it is in use by a thread, do nothing.
	while serialInUse:
		doNothing = 0

	# Yoink.
	serialInUse = 1

def serialCommExit():
	# Pull in the global variable serialInUse to deactivate it.
	global serialInUse

	# All done.
	serialInUse = 0

def initializeRobot():
	# Pull in the serial object and module number so we can initialize that number.
	global numModules
	global moduleType
	global moduleTwist
	global angleOffset
	global upstreamLength
	global downstreamLength
	global childPort
	global servoPower
	global servoAngles
	global servoSpeed
	global robotComm
	global parentHeight

	# Empty the arrays and variables that we pulled in from the global scope.
	numModules = 0
	moduleType = []
	moduleTwist = []
	angleOffset = []
	upstreamLength = []
	downstreamLength = []
	childPort = []
	servoPower = []
	servoAngles = []
	servoSpeed = []
	
	# Pad the front of the these arrays.
	moduleType.append(0)
	moduleTwist.append(0.0)
	angleOffset.append(0)
	upstreamLength.append(0.0)
	downstreamLength.append(parentHeight)
	servoPower.append(False)
	servoAngles.append(0.0)
	servoSpeed.append(0)

	# We currently only have one master port.
	childPort.append(0)

	# Create the command.
	command = "n;"

	# Grab the serial port.
	serialCommEnter()

	# Find out how many modules the robot has found.
	while not numModules:
		# Write the command to the robot.
		robotComm.write(command)

		# Read the command from the robot.
		response = robotComm.readline()

		# Check the response string for validity and return the result.
		response = response_check(response)

		if int(response):
			numModules = int(response)

	# Have to flush the input here for no apparent reason or it only reads numModules from the buffer.
	robotComm.flushInput()

	# Find the module type values.
	for i in range(1,numModules+1):
		# Initialize these values to zero.
		moduleType.append(0)
		servoPower.append(False)
		servoAngles.append(0.0)

		# Find the module types for the range of numModules.
		command = "r," + str(i) + ",t;"
		while not moduleType[i]:
			# Write the command to the robot.
			robotComm.write(command)

			# Read the response from the robot.
			response = robotComm.readline()

			# Check the response string for validity and return the result.
			response = response_check(response)

			# Convert the results of the response format checks to int.
			moduleType[i] = int(response,10)

	# Find the module child port values.
	for i in range(1,numModules):
		childPort.append(0)

		# Now find the child ports for the range of numModules.
		command = "r," + str(i) + ",c;"

		while not childPort[i]:
			# Write the command to the robot.
			robotComm.write(command)

			# Read the response from the robot.
			response = robotComm.readline()

			# Check the response string for validity and return the result.
			response = response_check(response)

			# Convert the results of the response format checks to int.
			childPort[i] = int(response,10)

	# Add an extra child port to the end of the array to avoid misaligned calculations.
	childPort.append(1)

	# Release the serial port.
	serialCommExit()

	# Fill in the rest of the values since we know module type.
	for i in range(1,numModules+1):
		if moduleType[i] == 1:
			moduleTwist.append((childPort[i] - 1)*child1PortAngle)
			angleOffset.append(child1ServoOffset)
			upstreamLength.append(child1Upstream)
			downstreamLength.append(child1Downstream)
		else:
			moduleTwist.append(0.0)
			angleOffset.append(0)
			upstreamLength.append(0.0)
			downstreamLength.append(0.0)

	for i in range(1,numModules+1):
		print "Module %d to %d is a twist of %f degrees." % (i,i+1,moduleTwist[i])

# This function takes a string and makes sure it is in decimal before
# we convert it. Otherwise, we just send a string with a 0 in it.
def response_check(response):
	formattedResponse = '0'

	# If we have a valid response, we extract the numbers.
	for i in range(0,len(response)):
		if (response[i] >= '0') and (response[i] <= '9'):
			formattedResponse += response[i]

	return formattedResponse

# This function converts an integer position value from the servo to a floating
# point representation with the given amount of precision.
def convert_to_angle(intVal, precision):
	# Obtain our precision factor.
	precisionFactor = 10**precision

	# Find the raw floating point value.
	angle = (intVal/1023.0)*300.0

	# Convert the raw float to one with the given precision.
	angle = float(int(angle*precisionFactor))/precisionFactor

	return angle

# Converts a user's input angle from degrees to an integer that the servo likes.
def convert_from_angle(floatVal):
	# Return the integer value.
	return int((floatVal/300.0)*1023)

# Function that handles requests for servo angle.
def handle_get_servo_angle(req):
	# Pull our serial port object in from the global scope.
	global robotComm

	# Pull in our angles array so we can store the angle for error checking.
	global servoAngles

	# Pull in our angle offsets so we can correctly alter the angles.
	global angleOffset

	# Pull in our total number of modules so that we don't allow an invalid index.
	global numModules

	if (req.ID <= numModules) and (req.ID > 0):
		# Create the command.
		command = "r," + str(req.ID) + ",a;"

		# Grab the serial port.
		serialCommEnter()

		# Write the command to the robot.
		robotComm.write(command)

		# Read the command from the robot.
		response = robotComm.readline()

		# Release the serial port.
		serialCommExit()

		# Check the response string for validity and return the result.
		response = response_check(response)

		if int(response,10):
			# Convert the result of the response format check to float.
			tempAngle = convert_to_angle(int(response,10) - angleOffset[req.ID],2)

			servoAngles[req.ID] = tempAngle

		# Return the floating point angle.
		return GetServoAngleResponse(servoAngles[req.ID])
	else:
		return GetServoAngleResponse(servoAngles[0])

# Function that handles requests to send the servo to a specific angle.
def handle_set_servo_angle(req):
	# Pull our serial port object in from the global scope.
	global robotComm

	# Pull in our angles array so we can store the angle for error checking.
	global servoAngles

	# Pull in our angle offsets so we can correctly alter the angles.
	global angleOffset

	# Pull in our total number of modules so that we don't allow an invalid index.
	global numModules

	if (req.ID <= numModules) and (req.ID > 0):
		# Create the command.
		command = "w," + str(req.ID) + ",a," + str(convert_from_angle(req.Angle) + angleOffset[req.ID]) + ";"

		# Grab the serial port.
		serialCommEnter()

		# Write the command to the robot.
		robotComm.write(command)

		# Release the serial port.
		serialCommExit()

		# Store the angle.
		servoAngles[req.ID] = req.Angle

		# Return the floating point angle.
		return SetServoAngleResponse(1)
	else:
		return SetServoAngleResponse(0)

# Function that handles a request to get the servo power status. This is useful for when
# this node is launched after the robot has been in use (we can't assume no torque).
def handle_get_servo_power(req):
	# Pull our serial port object in from the global scope.
	global robotComm

	# Pull in our angles array so we can store the angle for error checking.
	global servoPower

	# Pull in our total number of modules so that we don't allow an invalid index.
	global numModules

	if (req.ID <= numModules) and (req.ID > 0):
		# Create the command.
		command = "r," + str(req.ID) + ",p;"

		# Grab the serial port.
		serialCommEnter()

		# Write the command to the robot.
		robotComm.write(command)

		# Read the command from the robot.
		response = robotComm.readline()

		# Release the serial port.
		serialCommExit()

		# Check the response string for validity and return the result.
		response = response_check(response)

		# Convert the result of the response format check to int.
		tempResponse = int(response,10)

		if (tempResponse == 0) or (tempResponse == 1):
			servoPower[req.ID] = tempResponse

		# Return success.
		return GetServoPowerResponse(servoPower[req.ID])
	else:
		return GetServoPowerResponse(0)

# This function sets the servo power on and off, which allows manual movement
# of the servo with all of the feedback enabled.
def handle_set_servo_power(req):
	# Pull our serial port object in from the global scope.
	global robotComm

	# Pull in our power array so we can read and write to it.
	global servoPower

	# Pull in our total number of modules so that we don't allow an invalid index.
	global numModules

	if (req.ID <= numModules) and (req.ID > 0):
		# Create the command.
		command = "w," + str(req.ID) + ",p," + str(req.Power) + ";"

		# Grab the serial port.
		serialCommEnter()

		# Write the torque command to the robot.
		robotComm.write(command)

		# Release the serial port.
		serialCommExit()

		# Set the power value for this servo.
		servoPower[req.ID] = req.Power

		# Return a success.
		return SetServoPowerResponse(1)
	else:
		# Return a failure.
		return SetServoPowerResponse(0)

# Function that handles a request to get the servo speed value.
def handle_get_servo_speed(req):
	# Pull our serial port object in from the global scope.
	global robotComm

	# Pull in our speed percentages.
	global servoSpeed

	# Pull in our total number of modules so that we don't allow an invalid index.
	global numModules

	if (req.ID <= numModules) and (req.ID > 0):
		return GetServoSpeedResponse(servoSpeed[req.ID])
	else:
		return GetServoSpeedResponse(0)

# Function that handles a request to set the servo speed value.
def handle_set_servo_speed(req):
	# Pull our serial port object in from the global scope.
	global robotComm

	# Pull in our servo speed array so we can read and write to it.
	global servoSpeed

	# Pull in our total number of modules so that we don't allow an invalid index.
	global numModules

	if (req.ID <= numModules) and (req.ID > 0):
		# Grab the speed value.
		speedPercent = req.Speed

		# Convert that value to an integer between 1 and 1023 (10-bit minus the 0).
		speedValue = int(1022*(speedPercent/100.0) + 1)

		# Create the command.
		command = "w," + str(req.ID) + ",s," + str(speedValue) + ";"

		# Grab the serial port.
		serialCommEnter()

		# Write the torque command to the robot.
		robotComm.write(command)

		# Release the serial port.
		serialCommExit()

		# Set the power value for this servo.
		servoPower[req.ID] = req.Speed

		# Return a success.
		return SetServoSpeedResponse(1)
	else:
		# Return a failure.
		return SetServoSpeedResponse(0)

# Function to get the x,y,z position of the arm tip.
def handle_get_arm_tip(req):
	# Pull our serial port object in from the global scope.
	global robotComm

	# Pull in our angles array so we can store the angle for error checking.
	global servoAngles

	# Pull in our angle offsets so we can correctly alter the angles.
	global angleOffset

	# Pull in the upstream lengths of all modules.
	global upstreamLength

	# Pull in the downstream lengths of all modules.
	global downstreamLength

	# Pull in the twist angles of all modules.
	global moduleTwist

	# Pull in our total number of modules so that we don't allow an invalid index.
	global numModules

	# These are the floating point result values.
	x = 0.0
	y = 0.0
	z = 0.0

	# Update all of the servo angles first.
	for i in range(1,numModules+1):
		# Create the command.
		command = "r," + str(i) + ",a;"

		# Grab the serial port.
		serialCommEnter()

		# Write the command to the robot.
		robotComm.write(command)

		# Read the command from the robot.
		response = robotComm.readline()

		# Release the serial port.
		serialCommExit()

		# Check the response string for validity and return the result.
		response = response_check(response)

		if int(response,10):
			# Convert the result of the response format check to float.
			tempAngle = convert_to_angle(int(response,10) - angleOffset[i],2)

			servoAngles[i] = tempAngle

	# The result starts as the transform from the desired board testing plane to the arm plane.
	result_transform = [[0,0,-1,0],[0,1,0,0],[1,0,0,0],[0,0,0,1]]

	# Calculate the transform matrix.
	for i in range(1,numModules+1):
		# Create the information for the next transform in the chain.
		M = [[math.cos(degToRad(servoAngles[i])),
                      -1*math.sin(degToRad(servoAngles[i])),
                      0,
                      upstreamLength[i]+downstreamLength[i-1]],
		     [math.sin(degToRad(servoAngles[i]))*math.cos(degToRad(moduleTwist[i-1])),
                      math.cos(degToRad(servoAngles[i]))*math.cos(degToRad(moduleTwist[i-1])),
                      -1*math.sin(degToRad(moduleTwist[i-1])),
                      0],
		     [math.sin(degToRad(servoAngles[i]))*math.sin(degToRad(moduleTwist[i-1])),
                      math.cos(degToRad(servoAngles[i]))*math.sin(degToRad(moduleTwist[i-1])),
                      math.cos(degToRad(moduleTwist[i-1])),
                      0],
		     [0,0,0,1]]

		# Temporary matrix for storing matrix multiply results.
		temp_transform = []

		# Multiply two adjacent matrices together.
		for j in range(0,len(result_transform[0])):
			# Start a new row.
			temp_transform.append([])

			# Fill the row with values.
			for k in range(0,len(result_transform[0])):
				tempValue = 0.0
				for l in range(0,len(result_transform[0])):
					tempValue += result_transform[j][l]*M[l][k]
				temp_transform[j].append(tempValue)

		# Copy the temporary matrix result into the final result matrix.
		for j in range(0,len(result_transform[0])):
			for k in range(0,len(result_transform[0])):
				result_transform[j][k] = temp_transform[j][k]

	# Get the transformed values.
	x = result_transform[0][0]*downstreamLength[numModules] + result_transform[0][3]
	y = result_transform[1][0]*downstreamLength[numModules] + result_transform[1][3]
	z = result_transform[2][0]*downstreamLength[numModules] + result_transform[2][3]
	
	return GetArmTipResponse(x,y,z)

# This function returns the module total to whoever wants to know.
def handle_get_module_total(req):
	# Pull in the module number.
	global numModules

	return GetModuleTotalResponse(numModules)

# This function resets the robot.
def handle_reset_robot(req):
	# Pull in the module number.
	global numModules
	# Pull in the serial object.
	global robotComm

	# Create the command.
	command = "x;"

	# Grab the serial port.
	serialCommEnter()

	# Write the command to the robot.
	robotComm.write(command)

	# Release the serial port.
	serialCommExit()

	# Initialize the robot again.
	initializeRobot()

	return ResetRobotResponse(1)

# The following functions simply start their respective services.
def get_servo_angle_server():
	# Start the poll angle service.
	s = rospy.Service('get_servo_angle', GetServoAngle, handle_get_servo_angle)

def get_servo_power_server():
	# Start the poll angle service.
	s = rospy.Service('get_servo_power', GetServoPower, handle_get_servo_power)

def get_servo_speed_server():
	# Start the get speed service.
	s = rospy.Service('get_servo_speed', GetServoSpeed, handle_get_servo_speed)

def set_servo_angle_server():
	# Start the set angle service.
	s = rospy.Service('set_servo_angle', SetServoAngle, handle_set_servo_angle)

def set_servo_power_server():
	# Start the handle servo power service.
	s = rospy.Service('set_servo_power', SetServoPower, handle_set_servo_power)

def set_servo_speed_server():
	# Start the set speed service.
	s = rospy.Service('set_servo_speed', SetServoSpeed, handle_set_servo_speed)

def get_module_lengths_server():
	# Start the poll angle service.
	s = rospy.Service('get_module_lengths', GetModuleLengths, handle_get_module_lengths)

def get_module_offset_server():
	# Start the poll angle service.
	s = rospy.Service('get_module_offset', GetModuleOffset, handle_get_module_offset)

def get_module_twist_server():
	# Start the poll angle service.
	s = rospy.Service('get_module_twist', GetModuleTwist, handle_get_module_twist)

def get_module_total_server():
	# Start the poll angle service.
	s = rospy.Service('get_module_total', GetModuleTotal, handle_get_module_total)

def get_arm_tip_server():
	# Start the forward kinematic solver service.
	s = rospy.Service('get_arm_tip', GetArmTip, handle_get_arm_tip)

def reset_robot_server():
	# Start the software reset service.
	s = rospy.Service('reset_robot', ResetRobot, handle_reset_robot)

def start_robot_database():
	#Initialize the robot.
	initializeRobot()

	# Initialize the server.
	rospy.init_node('robot_data_server')

	# Start the services.
	get_servo_angle_server()
	set_servo_angle_server()
	get_servo_power_server()
	set_servo_power_server()
	get_servo_speed_server()
	set_servo_speed_server()
	get_module_total_server()
	get_arm_tip_server()
	reset_robot_server()

# Here are our global data variables...
# 
# An array that holds float values for servo angles.
servoAngles = []
# An array that holds whether a module is on or off.
servoPower = []
# An array that holds all of the servo speeds.
servoSpeed = []
# An array that holds all upstream module lengths (mm).
upstreamLength = []
# An array that holds all upstream module lengths (mm).
downstreamLength = []
# An array that holds the integer offsets required for module center.
angleOffset = []
# An array that holds the port number that each module's child is attached to.
childPort = []
# An array that holds the floating point angle value of the module twists.
moduleTwist = []
# An array that holds the integer values of all module types.
moduleType = []
# The number of modules this system has.
numModules = 0
# The height of the parent module (mm).
parentHeight = 67.0
# The length of the downstream portion of a child module type 1 (mm).
child1Downstream = 56.0
# The length of the upstream portion of a child module type 1 (mm).
child1Upstream = 51
# The angle of rotation between ports for child module type 1 (degrees).
child1PortAngle = 90.0
# The angle offset for the servo inside of child module type 1 (10-bit value).
child1ServoOffset = 511

# Use this variable to avoid serial comm errors from multiple threads.
serialInUse = 0

# This is our serial communication object.
robotComm = serial.Serial(port = '/dev/ttyUSB0',\
                          baudrate = 115200,\
                          bytesize = 8,\
                          parity = 'N',\
                          stopbits = 1,\
                          timeout = 0.01)

if __name__ == "__main__":
	# Start all of this database's services and functions.
	start_robot_database()

	# Start spinning and waiting for new data.
	rospy.spin()
