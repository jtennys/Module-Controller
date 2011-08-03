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
	target = sys.argv[4]
	numFails = 0
	command = ""
	
	robotComm.timeout = timeoutVal
	
	# Create command.
	if target[0] == 'S':
		command = "r," + str(targetModule) + ",a;"
	elif target[0] == 'C':
		command = "r," + str(targetModule) + ",t;"
	
	# Test the target in a sweep for totalTrials.
	for i in range(0,totalTrials):
		# Write the command to the robot.
		robotComm.write(command)

		# Read the command from the robot.
		response = robotComm.readline()

		# Check the response string for validity and return the result.
		response = response_check(response)

		if not int(response,10):
			numFails += 1

		time.sleep(0.01)

	print "\n"
	print "Success Rate: %f%%" % (100.0*(float(totalTrials-numFails)/float(totalTrials)))
	print "\n"



