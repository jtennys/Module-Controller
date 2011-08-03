#!/usr/bin/env python
import roslib; roslib.load_manifest('module_controller')
from module_controller.srv import *
import rospy
import string
import time
import sys

if __name__ == "__main__":
	knownModuleTotal = int(sys.argv[1])
	totalTrials = int(sys.argv[2])
	numFails = []
	trialTime = []
	minTrialTime = 0.0
	maxTrialTime = 0.0
	totalTrialTime = 0.0
	totalFails = 0

	# Populate the numfails list with zeros.
	for i in range(0,knownModuleTotal+1):
		numFails.append(0)
	
	# Offset the trial times so that the trial number is the index value.
	trialTime.append(0.0)

	# Loop for the total number of trials and reset/get total over and over again.
	for i in range(1,totalTrials+1):
		rospy.wait_for_service('reset_robot')

		# Get the start time.
		startTime = time.time()

		try:
			reset_robot = rospy.ServiceProxy('reset_robot', ResetRobot)
			response = reset_robot(1)
			if response.Success:
				rospy.wait_for_service('get_module_total')
				try:
					get_module_total = rospy.ServiceProxy('get_module_total', GetModuleTotal)
					response = get_module_total(1)
					if response.Total <= knownModuleTotal:
						numFails[response.Total] += 1
				except rospy.ServiceException, e:
					print "Service call failed: %s" % e
			else:
				print "Failed to reset!"
		except rospy.ServiceException, e:
			print "Service call failed: %s" % e

		# Get the end time.
		endTime = time.time()

		# The trial time is the time from after the reset wait to completion of the num total response.
		trialTime.append(endTime - startTime)

		# Keep a running total of the trial time.
		totalTrialTime += trialTime[i]

		# Update the min trial time.
		if(minTrialTime > 0.0):
			if(trialTime[i] < minTrialTime):
				minTrialTime = trialTime[i]
		else:
			minTrialTime = trialTime[i]

		# Update the max trial time.
		if(maxTrialTime > 0.0):
			if(trialTime[i] > maxTrialTime):
				maxTrialTime = trialTime[i]
		else:
			maxTrialTime = trialTime[i]

	print "\n\nResults after %d trials:" % totalTrials

	print "\nMinimum trial time: %.4f seconds" % minTrialTime
	print "Maximum trial time: %.4f seconds" % maxTrialTime
	print "Average trial time: %.4f seconds\n" % (totalTrialTime/float(totalTrials))
	
	for i in range(0,knownModuleTotal):
		print "Failures where %d modules were found: %d" % (i,numFails[i])
		totalFails = totalFails + numFails[i]

	print "\nSuccess: %f%%\n\n" % (100.0*(float(numFails[knownModuleTotal])/float(totalTrials)))

