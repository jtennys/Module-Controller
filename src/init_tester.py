#!/usr/bin/env python
import roslib; roslib.load_manifest('module_controller')
from module_controller.srv import *
import rospy
import string
import sys

if __name__ == "__main__":
	knownModuleTotal = int(sys.argv[1])
	totalTrials = int(sys.argv[2])
	numFails = []
	totalFails = 0

	for i in range(0,knownModuleTotal+1):
		numFails.append(0)
	
	for i in range(1,totalTrials+1):
		rospy.wait_for_service('reset_robot')
		try:
			reset_robot = rospy.ServiceProxy('reset_robot', ResetRobot)
			response = reset_robot(1)
			if response.Success:
				rospy.wait_for_service('get_module_total')
				try:
					get_module_total = rospy.ServiceProxy('get_module_total', GetModuleTotal)
					response = get_module_total(1)
					if response.Total <= knownModuleTotal:
						numFails[response.Total] = numFails[response.Total] + 1
				except rospy.ServiceException, e:
					print "Service call failed: %s" % e
			else:
				print "Failed to reset!"
		except rospy.ServiceException, e:
			print "Service call failed: %s" % e

	print "\n\nResults after %d trials:" % totalTrials

	for i in range(0,knownModuleTotal):
		print "Failures where %d modules were found: %d" % (i,numFails[i])
		totalFails = totalFails + numFails[i]

	print "Success: %f%%\n\n" % (100.0*(float(numFails[knownModuleTotal])/float(totalTrials)))
