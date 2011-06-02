#!/usr/bin/env python
import roslib; roslib.load_manifest('module_controller')
from module_controller.srv import *
import rospy
import serial
import threading
import time

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

# This function takes a string and makes sure it is in decimal before
# we convert it. Otherwise, we just send a string with a 0 in it.
def response_check(response):
	global commFails
	global MAX_COMM_FAILS
	global robotComm

	formattedResponse = '0'

	if len(response):
		# Clear the number of communication failures to 0.
		commFails = 0

		# If we have a valid response, we extract the numbers.
		for i in range(0,len(response)):
			if (response[i] >= '0') and (response[i] <= '9'):
				formattedResponse += response[i]
	else:
		# Increment the number of communication failures.
		commFails += 1

		if commFails >= MAX_COMM_FAILS:
			robotComm.flushInput()
			robotComm.flushOutput()

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

# This function handles a request by a computation node to get a module's length.
def handle_get_module_lengths(req):
	# Pull in the length arrays and module number.
	global upstreamLength
	global downstreamLength
	global numModules

	if (req.ID <= numModules) and (req.ID > 0):
		# Return the values.
		return GetModuleLengthsResponse(upstreamLength[req.ID],downstreamLength[req.ID])
	else:
		return GetModuleLengthsResponse(0,0)

# This function handles a request by a computation node to get a module's zero angle offset.
def handle_get_module_offset(req):
	# Pull in the length arrays and module number.
	global angleOffset
	global numModules

	if (req.ID <= numModules) and (req.ID > 0):
		# Return the values.
		return GetModuleOffsetResponse(angleOffset[req.ID])
	else:
		return GetModuleOffsetResponse(0)

# This function handles a request by a computation node to get a module's link twist.
def handle_get_module_twist(req):
	# Pull in the length arrays and module number.
	global moduleTwist
	global numModules

	if (req.ID < numModules) and (req.ID > 0):
		# Return the values.
		return GetModuleTwistResponse(moduleTwist[req.ID])
	else:
		return GetModuleTwistResponse(0)

# This function returns the module total to whoever wants to know.
def handle_get_module_total(req):
	# Pull in the module number.
	global numModules

	return GetModuleTotalResponse(numModules)

# The following functions simply start their respective services.
def get_servo_angle_server():
	# Start the poll angle service.
	s = rospy.Service('get_servo_angle', GetServoAngle, handle_get_servo_angle)

def get_servo_power_server():
	# Start the poll angle service.
	s = rospy.Service('get_servo_power', GetServoPower, handle_get_servo_power)

def set_servo_angle_server():
	# Start the set angle service.
	s = rospy.Service('set_servo_angle', SetServoAngle, handle_set_servo_angle)

def set_servo_power_server():
	# Start the handle servo power service.
	s = rospy.Service('set_servo_power', SetServoPower, handle_set_servo_power)

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
	
def start_robot_database():
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
	global robotComm
	global commFails

	# Pad the front of the these arrays.
	moduleType.append(0)
	moduleTwist.append(0.0)
	angleOffset.append(0)
	upstreamLength.append(0.0)
	downstreamLength.append(0.0)
	servoPower.append(False)
	servoAngles.append(0.0)

	# We currently only have one master port.
	childPort.append(0)

	# Create the command.
	command = "n;"

	# Grab the serial port.
	serialCommEnter()

	# Find out how many modules the robot has found.
	while not numModules:
		# Flush the output buffer and write the command to the robot.
		robotComm.flushOutput()
		robotComm.write(command)

		# Read the command from the robot and flush the input buffer.
		response = robotComm.readline()
		robotComm.flushInput()

		# Check the response string for validity and return the result.
		response = response_check(response)

		# Clear comm fails because we don't want to fail out waiting for the robot to turn on.
		commFails = 0

		if int(response):
			numModules = int(response)

	# Find the module type and child port values.
	for i in range(1,numModules+1):
		# Initialize these values to zero.
		moduleType.append(0)
		servoPower.append(False)
		servoAngles.append(0.0)

		# Find the module types for the range of numModules.
		command = "r," + str(i) + ",t;"
		while not moduleType[i]:
			# Flush the output buffer and write the command to the robot.
			robotComm.flushOutput()
			robotComm.write(command)

			# Read the command from the robot and flush the input buffer.
			response = robotComm.readline()
			robotComm.flushInput()

			# Check the response string for validity and return the result.
			response = response_check(response)

			# Convert the results of the response format checks to int.
			moduleType[i] = int(response,10)

	for i in range(1,numModules):
		childPort.append(0)

		# Now find the child ports for the range of numModules.
		command = "r," + str(i) + ",c;"

		while not childPort[i]:
			# Flush the output buffer and write the command to the robot.
			robotComm.flushOutput()
			robotComm.write(command)

			# Read the command from the robot and flush the input buffer.
			response = robotComm.readline()
			robotComm.flushInput()

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
			moduleTwist.append((childPort[i] - 1)*90.0)
			angleOffset.append(511)
			upstreamLength.append(50.0)
			downstreamLength.append(50.0)
		else:
			moduleTwist.append(0.0)
			angleOffset.append(0)
			upstreamLength.append(0.0)
			downstreamLength.append(0.0)

	# Initialize the server.
	rospy.init_node('robot_data_server')

	for i in range(1,numModules+1):
		print "Module %d to %d is a twist of %f degrees." % (i,i+1,moduleTwist[i])
                print "Module %d is of type %d." % (i,moduleType[i])


	# Start the services.
	get_servo_angle_server()
	set_servo_angle_server()
	get_servo_power_server()
	set_servo_power_server()
	get_module_lengths_server()
	get_module_offset_server()
	get_module_twist_server()
	get_module_total_server()

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
# The number of continuous communication failures we have had.
commFails = 0
# The number of allowable communication failures before taking action.
MAX_COMM_FAILS = 50

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
