#!/usr/bin/env python
import roslib; roslib.load_manifest('module_controller')
import rospy
import serial

def get_robot_data():
	robotComm = serial.Serial(port = '/dev/ttyUSB0',\
                                  baudrate = 115200,\
                                  bytesize = 8,\
                                  parity = 'N',\
                                  stopbits = 1,\
                                  timeout = 1)

        command = "r,1,a;"

        while True:
                # Write the command to the robot.
                robotComm.write(command)

                # Read the command from the robot.
                response = robotComm.readline()

                # Check the response string for validity and return the result.
                response = response_check(response)

                # Convert the result of the response format check to float.
                angle = convert_angle(int(response,10),2)
                print angle

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

def handle_poll_servo_angle():
	print "HI"

def poll_servo_angle_server():
	rospy.init_node('pollServoAngleServer')
	s = rospy.Service('poll_servo_angle', PollServoAngle, handle_poll_servo_angle)
	rospy.spin()

if __name__ == "__main__":
	getRobotData()
	poll_servo_angle_server()
