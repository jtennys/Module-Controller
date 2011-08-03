#!/usr/bin/env python
import serial
import time
import sys
import csv

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
	# Pull in the roslaunch command line parameters.
	targetModule = int(sys.argv[1])
	totalTrials = int(sys.argv[2])
	startVal = float(sys.argv[3])/1000.0
	endVal = float(sys.argv[4])/1000.0
	stepVal = float(sys.argv[5])/1000.0
	regulated = int(sys.argv[6])

	servoFails = []
	controllerFails = []
	data = []
	dataTitles = []
	
	# Populate the data lists.
	servoFails.append(0)
	controllerFails.append(0)		
	data.append(0.0)
	dataTitles.append("Timeout")

	for i in range(1,targetModule+1):
		servoFails.append(0)
		controllerFails.append(0)		
		data.append(0.0)
		dataTitles.append("Servo " + str(i))

	for i in range(1,targetModule+1):
		data.append(0.0)
		dataTitles.append("Cont " + str(i))

	# Create a csv spreadsheet writer object and initialize it.
	resultWriter = csv.writer(open('/home/jason/ros_packages/module_controller/test.csv', 'wb'), delimiter=' ')
	resultWriter.writerow(dataTitles)

	# If the start value is 0, fill in all zero values and increment.
	if startVal == 0.0:
		resultWriter.writerow(data)
		startVal += stepVal

	while startVal <= endVal:
		robotComm.timeout = startVal
		data[0] = startVal*1000

		for i in range(0,targetModule+1):
			servoFails[i] = 0
			controllerFails[i] = 0

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

				if regulated:
					time.sleep(0.01)
					robotComm.flushInput()

			data[i] = 100.0*(float(totalTrials-servoFails[i])/float(totalTrials))

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

				if regulated:
					time.sleep(0.01)
					robotComm.flushInput()

			data[i+targetModule] = 100.0*(float(totalTrials-controllerFails[i])/float(totalTrials))

		startVal += stepVal

		print "Timeout %.2f done!" % data[0]
		resultWriter.writerow(data)



