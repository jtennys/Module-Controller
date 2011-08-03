#!/usr/bin/env python
import serial
import time
import sys

# This function takes a string and makes sure it is in decimal before
# we convert it. Otherwise, we just send a string with a 0 in it.
def response_check(response):
	formattedResponse = '0'

	# If we have a valid response, we extract the numbers.
	for i in range(0,len(response)):
		if (response[i] >= '0') and (response[i] <= '9'):
			formattedResponse += response[i]

	return formattedResponse

# This is our serial communication object.
robotComm = serial.Serial(port = '/dev/ttyUSB0',\
                          baudrate = 115200,\
                          bytesize = 8,\
                          parity = 'N',\
                          stopbits = 1)

if __name__ == "__main__":
	targetModule = int(sys.argv[1])
	totalTrials = int(sys.argv[2])
	timeoutVal = float(sys.argv[3])/1000.0
	servoFails = []
	controllerFails = []
	
	for i in range(0,targetModule+1):
		servoFails.append(0)
		controllerFails.append(0)

	robotComm.timeout = timeoutVal
	
	for i in range(1,targetModule+1):
		command = "r," + str(i) + ",a;"

		# Test the target in a sweep for totalTrials.
		for j in range(0,totalTrials):
			# Write the command to the robot.
			robotComm.write(command)

			# Read the command from the robot.
			response = robotComm.readline()

			# Check the response string for validity and return the result.
			response = response_check(response)

			if not int(response,10):
				servoFails[i] += 1


			time.sleep(0.01)
			robotComm.flushInput()

		# time.sleep(0.1)

	for i in range(1,targetModule+1):
		command = "r," + str(i) + ",t;"

		# Test the target in a sweep for totalTrials.
		for j in range(0,totalTrials):
			# Write the command to the robot.
			robotComm.write(command)

			# Read the command from the robot.
			response = robotComm.readline()

			# Check the response string for validity and return the result.
			response = response_check(response)

			if not int(response,10):
				controllerFails[i] += 1

			time.sleep(0.01)
			robotComm.flushInput()

		# time.sleep(0.1)

	print "\n"
	for i in range(1,targetModule+1):
		print "Servo %d: %.1f%%" % (i,100.0*(float(totalTrials-servoFails[i])/float(totalTrials)))

	for i in range(1,targetModule+1):
		print "Controller %d: %.1f%%" % (i,100.0*(float(totalTrials-controllerFails[i])/float(totalTrials)))
	print "\n"



