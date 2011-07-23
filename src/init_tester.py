#!/usr/bin/env python
import roslib; roslib.load_manifest('module_controller')
from module_controller.srv import *
import rospy
import string
import sys

if __name__ == "__main__":
	user_in = ""
	knownModuleTotal = 2
	numFails = 0
	numTrials = 0
	totalTrials = 1000
	
	for i in range(1,totalTrials+1):
		rospy.wait_for_service('reset_robot', 1)
		try:
			reset_robot = rospy.ServiceProxy('reset_robot', ResetRobot)
			response = reset_robot(1)
			if response.Success:
				rospy.wait_for_service('get_module_total')
				try:
					get_module_total = rospy.ServiceProxy('get_module_total', GetModuleTotal)
					response = get_module_total(1)
					if response.Total != knownModuleTotal:
						numFails = numFails + 1
				except rospy.ServiceException, e:
					print "Service call failed: %s" % e
			else:
				print "Failed to reset!"
		except rospy.ServiceException, e:
			print "Service call failed: %s" % e
			
	print "Init success after %d trials: %f%%" % (totalTrials,100.0*(float(totalTrials-numFails)/float(totalTrials)))
