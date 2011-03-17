#!/usr/bin/env python
import roslib; roslib.load_manifest('module_controller')
from module_controller.srv import *
import rospy
import serial
import threading
import time

def serialCommEnter():
	global serialInUse

	while serialInUse:
		doNothing = 0

	serialInUse = 1

def serialCommExit():
	global serialInUse

	serialInUse = 0

def response_check(response):
	formattedResponse = '0'

	# If we have a valid response, we extract the numbers.
	for i in range(0,len(response)):
		if (response[i] >= '0') and (response[i] <= '9'):
			formattedResponse += response[i]

	return formattedResponse

def convert_to_angle(intVal, precision):
	# Obtain our precision factor.
	precisionFactor = 10**precision

	# Find the raw floating point value.
	angle = (intVal/1023.0)*300.0

	# Convert the raw float to one with the given precision.
	angle = float(int(angle*precisionFactor))/precisionFactor

	return angle

def convert_from_angle(floatVal):
	# Return the integer value.
	return int((floatVal/300.0)*1023)

def handle_get_servo_angle(req):
	# Pull our serial port object in from the global scope.
	global robotComm

	# Pull in our angles array so we can store the angle for error checking.
	global servoAngles

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

		# Convert the result of the response format check to float.
		tempAngle = convert_to_angle(int(response,10),2)

		# If we have a nonzero value, store it. Otherwise, return the previous good value.
		if tempAngle:
			servoAngles[req.ID] = tempAngle

		# Return the floating point angle.
		return GetServoAngleResponse(servoAngles[req.ID])
	else:
		return GetServoAngleResponse(servoAngles[0])

def handle_set_servo_angle(req):
	# Pull our serial port object in from the global scope.
	global robotComm

	# Pull in our angles array so we can store the angle for error checking.
	global servoAngles

	# Pull in our total number of modules so that we don't allow an invalid index.
	global numModules

	if (req.ID <= numModules) and (req.ID > 0):
		# Create the command.
		command = "w," + str(req.ID) + ",a," + str(convert_from_angle(req.Angle)) + ";"

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

def get_servo_angle_server():
	# Pull in the global module values for initialization.
	global servoAngles
	global numModules

	# Append zero values for all possible module IDs.
	for i in range(0,numModules+1):
		servoAngles.append(0.0)

	# Start the poll angle service.
	s = rospy.Service('get_servo_angle', GetServoAngle, handle_get_servo_angle)

def get_servo_power_server():
	# Pull in the power boolean array and module number.
	global servoPower
	global numModules

	# Append zero values for all possible module IDs.
	for i in range(0,numModules+1):
		servoPower.append(False)

	# Start the poll angle service.
	s = rospy.Service('get_servo_power', GetServoPower, handle_get_servo_power)

def set_servo_angle_server():
	# Start the set angle service.
	s = rospy.Service('set_servo_angle', SetServoAngle, handle_set_servo_angle)

def set_servo_power_server():
	# Start the handle servo power service.
	s = rospy.Service('set_servo_power', SetServoPower, handle_set_servo_power)
	
def start_robot_database():
	# Pull in the serial object and module number so we can initialize that number.
	global numModules
	global robotComm

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

	# Release the serial port.
	serialCommExit()

	# Initialize the server.
	rospy.init_node('robot_data_server')

	# Start the services.
	get_servo_angle_server()
	set_servo_angle_server()
	get_servo_power_server()
	set_servo_power_server()

# Here are our global data variables...
# 
# An array that holds float values for servo angles.
servoAngles = []
# An array that holds whether a module is on or off.
servoPower = []
# An array that holds all of the servo speeds.
servoSpeed = []
# The number of modules this system has.
numModules = 0

# Use this variable to avoid serial comm errors from multiple threads.
serialInUse = 0

# This is our serial communication object.
robotComm = serial.Serial(port = '/dev/ttyUSB0',\
                          baudrate = 115200,\
                          bytesize = 8,\
                          parity = 'N',\
                          stopbits = 1,\
                          timeout = 0.005)

if __name__ == "__main__":
	# Start all of this database's services and functions.
	start_robot_database()

	# Start spinning and waiting for new data.
	rospy.spin()
