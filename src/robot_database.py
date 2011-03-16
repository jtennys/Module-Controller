#!/usr/bin/env python
import roslib; roslib.load_manifest('module_controller')
from module_controller.srv import *
import rospy
import serial
import threading
import time

class get_servo_angles(threading.Thread):
	# Open a serial connection to the robot.
        robotComm = serial.Serial(port = '/dev/ttyUSB0',\
                                  baudrate = 115200,\
                                  bytesize = 8,\
                                  parity = 'N',\
                                  stopbits = 1,\
                                  timeout = 0.01)

	def __init__(self):
		# Allow the use of this class as a thread.
		threading.Thread.__init__(self)

		# Pull in the necessary variables from the global scope.
		global servoAngles
		global numModules

		# Append an empty angle to the front of the servoAngles array
		# to offset the index so that module IDs line up.
		servoAngles.append(0.0)

		# Find out how many modules the robot has found.
		numModules = 3

		# Append zero values for all possible module IDs.
		for i in range(1,numModules+1):
			servoAngles.append(0.0)

	def run(self):
		txSent = 0

	        while not rospy.is_shutdown():
			for i in range(1,numModules+1):
				# Create the command.
				command = "r," + str(i) + ",a;"

		                # Write the command to the robot.
		                self.robotComm.write(command)

		                # Read the command from the robot.
		                response = self.robotComm.readline()

		                # Check the response string for validity and return the result.
		                response = response_check(response)

		                # Convert the result of the response format check to float.
				tempAngle = convert_angle(int(response,10),2)

				if tempAngle:
		                	servoAngles[i] = tempAngle
					print "Servo %d: %s" % (i,servoAngles[i])

#				time.sleep(0.1)

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

def handle_poll_servo_angle(req):
	# Send back the servo angle.
	return PollServoAngleResponse(str(servoAngles[req.ID]))

def poll_servo_angle_server():
	# Initialize the server
	rospy.init_node('poll_servo_angle_server')
	s = rospy.Service('poll_servo_angle', PollServoAngle, handle_poll_servo_angle)
	rospy.spin()

# Here are our global data variables.
servoAngles = []
numModules = 0

if __name__ == "__main__":
	# Create the thread object that gets servo angle data.
	angleGatherer = get_servo_angles()

	# Start the thread object that gets servo angle data.
	angleGatherer.start()

	# Start the servo angle service.
	poll_servo_angle_server()
