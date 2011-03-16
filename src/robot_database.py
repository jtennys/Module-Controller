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
	# If we have a valid response, we extract the numbers.
        if len(response) >= 3:
                response = response[1:-1]
        else:
                response = '0'

        # This variable is created as a check that this is decimal.
        valid = True

        # If all numbers are base 10, we exit with valid as True.
        for i in range(0,len(response)):
                if (response[i] < '0') or (response[i] > '9'):
                        valid = False

	# If the response is not valid, set the response to 0.
	if not valid:
		response = '0'

	return response

def convert_angle(intVal, precision):
	# Obtain our precision factor.
	precisionFactor = 10**precision

	# Find the raw floating point value.
	angle = (intVal/1023.0)*300.0

	# Convert the raw float to one with the given precision.
	angle = float(int(angle*precisionFactor))/precisionFactor

	return angle

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
		tempAngle = convert_angle(int(response,10),2)

		# If we have a nonzero value, store it. Otherwise, return the previous good value.
		if tempAngle:
			servoAngles[req.ID] = tempAngle

		# Return the floating point angle.
		return GetServoAngleResponse(servoAngles[req.ID])
	else:
		return GetServoAngleResponse(servoAngles[0])

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

def set_servo_power_server():
	# Pull in the power boolean array and module number.
	global servoPower
	global numModules

	# Append zero values for all possible module IDs.
	for i in range(0,numModules+1):
		servoPower.append(False)

	# Start the handle servo power service.
	s = rospy.Service('set_servo_power', SetServoPower, handle_set_servo_power)
	
def start_robot_database():
	# Pull in the serial object and module number so we can initialize that number.
	global numModules
	global robotComm

	# Find out how many modules the robot has found.
	numModules = 3

	# Initialize the server.
	rospy.init_node('robot_data_server')

	# Start the services.
	get_servo_angle_server()

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
