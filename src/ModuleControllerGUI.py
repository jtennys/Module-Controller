#!/usr/bin/env python
import roslib; roslib.load_manifest('module_controller')
from module_controller.srv import *
import rospy
import threading
import time
import string

import pygtk
pygtk.require('2.0')
import gtk

# Initialize the gtk thread engine.
gtk.gdk.threads_init()

# This class is an instance (a window) of the modular robot controller GUI.
class ModuleControllerGUI(threading.Thread):
	# Create the killthread event for stopping the thread.
	killthread = threading.Event()

	# This function is called when this object is "started".
	def run(self):
		global buttonList
		global textBoxList
		global numModules
		global angleUnit
		
		# Initialize the power toggle boxes.
		for i in range(1,numModules+1):
			# Get the current power status for all servos.
			rospy.wait_for_service('get_servo_power', 1)
			try:
				get_servo_power = rospy.ServiceProxy('get_servo_power', GetServoPower)
				response = get_servo_power(i)

				gtk.gdk.threads_enter()
				buttonList[i].set_active(response.Power)
				gtk.gdk.threads_leave()
			except rospy.ServiceException, e:
				print "Service call failed: %s" % e

		# Initialize the servo angles.
		for i in range(1,numModules+1):
			# Get the current angle for all servos.
			rospy.wait_for_service('get_servo_angle', 1)
			try:
				get_servo_angle = rospy.ServiceProxy('get_servo_angle', GetServoAngle)
				response = get_servo_angle(i)
				angle = 0.0
				if cmp(angleUnit,"radians") == 0:
					angle = response.Angle*(3.14159/180.0)
				else:
					angle = response.Angle

				gtk.gdk.threads_enter()
				textBoxList[i].set_text(str(angle))
				gtk.gdk.threads_leave()
			except rospy.ServiceException, e:
				print "Service call failed: %s" % e


		# While the user hasn't closed the window or pressed ctrl+c.
		while not (self.killthread.isSet() or rospy.is_shutdown()):
			for i in range(1,numModules+1):
				# Don't print to an active servo box. This is done
				# to avoid overwriting user input.
				if buttonList[i].get_active() == False:
					rospy.wait_for_service('get_servo_angle', 1)
					try:
						get_servo_angle = rospy.ServiceProxy('get_servo_angle', GetServoAngle)
						response = get_servo_angle(i)
						angle = 0.0
						if cmp(angleUnit,"radians") == 0:
							angle = response.Angle*(3.14159/180.0)
						else:
							angle = response.Angle

						gtk.gdk.threads_enter()
						textBoxList[i].set_text(str(angle))
						gtk.gdk.threads_leave()
					except rospy.ServiceException, e:
						print "Service call failed: %s" % e

			# Don't print to distance boxes if they're editable. This is done
			# to avoid overwriting user input.
			if buttonList[0].get_active() == False:
				rospy.wait_for_service('get_servo_angle', 1)
				try:
					get_arm_tip = rospy.ServiceProxy('get_arm_tip', GetArmTip)
					response = get_arm_tip(1)
					x = response.x
					y = response.y
					z = response.z

					# All distances come from the database in mm and must be converted.
					if cmp(distanceUnit,"feet") == 0:
						x = x*0.0032808399
						y = y*0.0032808399
						z = z*0.0032808399
					elif cmp(distanceUnit,"inches") == 0:
						x = x*0.0393700787
						y = y*0.0393700787
						z = z*0.0393700787
					elif cmp(distanceUnit,"meters") == 0:
						x = x*0.001
						y = y*0.001
						z = z*0.001
					elif cmp(distanceUnit,"centimeters") == 0:
						x = x*0.1
						y = y*0.1
						z = z*0.1

					# Obtain access to the distance text boxes and print the values.
					gtk.gdk.threads_enter()
					textBoxList[numModules+1].set_text(str(x))
					textBoxList[numModules+2].set_text(str(y))
					textBoxList[numModules+3].set_text(str(z))
					gtk.gdk.threads_leave()
				except rospy.ServiceException, e:
					print "Service call failed: %s" % e

	# Set the killthread variable.
	def stop(self):
		self.killthread.set()
		
	# Function that is called when a click action happens.
	def callback(self, widget, data=None):
		global buttonList
		global textBoxList
		global numModules
		global notAll

		# If "All" has been checked, check all boxes.
		if cmp(data,"Power All") == 0:
			# If this is not a request to just toggle the "All" box
			if notAll:
				if widget.get_active() == True:
					# If "All" has been clicked on, check everything.
					for i in range(1,numModules+1):
						buttonList[i].set_active(True)

					# Enable x y z entry since all modules are on.
					for j in range(1,4):
						textBoxList[numModules+j].set_editable(True)

					# Enable the "Send Coordinates" button.
					buttonList[numModules+1].set_sensitive(True)
					
				elif widget.get_active() == False:
					# If "All" has been clicked off, uncheck everything.
					for i in range(1,numModules+1):
						buttonList[i].set_active(False)
		elif cmp(data, "Send Coordinates") == 0:
			if buttonList[0].get_active():
				print textBoxList[numModules+1].get_text()
				print textBoxList[numModules+2].get_text()
				print textBoxList[numModules+3].get_text()
		elif cmp(data, "Reset Arm") == 0:
			rospy.wait_for_service('reset_robot', 1)
			try:
				reset_robot = rospy.ServiceProxy('reset_robot', ResetRobot)
				response = reset_robot(1)
				if response.Success:
					print "Robot Reset"
			except rospy.ServiceException, e:
				print "Service call failed: %s" % e
		elif cmp(data[:5], "Speed") == 0:
			if data[6] == 'A':
				# Grab the input percentage value.
				percentAll = widget.get_value()
				for i in range(1, numModules+1):
					rospy.wait_for_service('set_servo_speed', 1)
					try:
						set_servo_speed = rospy.ServiceProxy('set_servo_speed', SetServoSpeed)
						response = set_servo_speed(i,percentAll)
					except rospy.ServiceException, e:
						print "Service call failed: %s" % e
			else:
				# Grab the index of the input slider.
				index = int(data[6:len(data)])
				# Grab the input percentage value.
				percent = widget.get_value()
				rospy.wait_for_service('set_servo_speed', 1)
				try:
					set_servo_speed = rospy.ServiceProxy('set_servo_speed', SetServoSpeed)
					response = set_servo_speed(index,percent)
				except rospy.ServiceException, e:
					print "Service call failed: %s" % e
				
		elif cmp(data, "Save Pose") == 0:
			tempStr = "/home/jason/ros_packages/module_controller/src/"+textBoxList[numModules+4].get_text()+".pos"
			poseFile = open(tempStr, 'w')
			for i in range(1,numModules+1):
				if cmp(angleUnit,"radians") == 0:
					angle = float(textBoxList[i].get_text())*(180/3.14159)
					poseFile.write(str(angle)+"\n")
				else:
					poseFile.write(textBoxList[i].get_text()+"\n")

			poseFile.close()
		elif cmp(data, "Run Motion") == 0:
			# Open the motion file.
			tempStr = "/home/jason/ros_packages/module_controller/src/"+textBoxList[numModules+4].get_text()+".mot"
			motFile = open(tempStr, 'r')

			# Open the individual position file.
			tempStr = motFile.readline()
			pose = string.split(tempStr,',')
			tempStr = pose[0]
			tempStr.replace('\n','')
			print repr(tempStr)
			tempStr = "/home/jason/ros_packages/module_controller/src/"+"pose1"+".pos"
			posFile = open(tempStr, 'r')
			# Go to the specifiec pose.
			rospy.wait_for_service('set_servo_angle', 1)
			try:
				set_servo_angle = rospy.ServiceProxy('set_servo_angle', SetServoAngle)
				for i in range(1,numModules+1):
					angleStr = posFile.readline()
					response = set_servo_angle(i, float(angleStr))
					if response.Success:
						print "Set Module %d to %f degrees!" % (i,float(angleStr))
						textBoxList[i].set_text(angleStr)
						buttonList[i].set_active(True)
			except rospy.ServiceException, e:
				print "Service call failed: %s" % e
			posFile.close()
			motFile.close()

		else:
			# If a module box has been checked off, we have to look at
			# this condition to make sure that the "All" box also gets
			# checked off, since all modules are no longer on.
			if widget.get_active() == False:
				# We are now directing this command at the "All" box.
				notAll = False
				# Set the "All" button to false.
				buttonList[0].set_active(False)
				# Reset the notAll variable back to True.
				notAll = True
				# Disable x y z entry since not all modules are on.
				for i in range(1,4):
					textBoxList[numModules+i].set_editable(False)
				# Disable the "Send Coordinates" button.
				buttonList[numModules+1].set_sensitive(False)
			else:
				# Create a variable to check if all modules are on.
				# If they are, we will set the "All" box to true.
				allOn = True
				for i in range(1,numModules+1):
					if buttonList[i].get_active() == False:
						allOn = False
				
				if allOn:
					buttonList[0].set_active(True)

					# Enable the "Send Coordinates" button.
					buttonList[numModules+1].set_sensitive(True)
				
			# Set the correct text box to active when it is powered on.
			# Also, set the power to be on or off correctly.
			for i in range(1,numModules+1):
				if cmp(widget.get_label(),buttonList[i].get_label()) == 0:
					textBoxList[i].set_editable(widget.get_active())
					rospy.wait_for_service('set_servo_power', 0.1)
					try:
						set_servo_power = rospy.ServiceProxy('set_servo_power', SetServoPower)
						response = set_servo_power(i,widget.get_active())
						if response.Success:
							print "Servo %d %s!" % (i, ("OFF", "ON")[widget.get_active()])
					except rospy.ServiceException, e:
						print "Service call failed: %s" % e

	# Function that is called when an enter key is pressed in a text box.
	def enter_callback(self, widget, entry):
		# Pull in the button list array.
		global textBoxList
		global angleUnit

		# This is where entered angles are sent.  A condition is added
		# to force users to click the send coordinates button.
		if entry.get_editable():
			print widget.get_name()
			if (cmp(widget.get_name(),"X") != 0) and \
			   (cmp(widget.get_name(),"Y") != 0) and \
			   (cmp(widget.get_name(),"Z") != 0) and \
			   (cmp(widget.get_name(),"POSE") != 0):
				rospy.wait_for_service('set_servo_angle', 1)
				try:
					set_servo_angle = rospy.ServiceProxy('set_servo_angle', SetServoAngle)
					for i in range(1,numModules+1):
						if cmp(widget.get_name(),textBoxList[i].get_name()) == 0:
							angle = 0.0

							if cmp(angleUnit,"radians") == 0:
								angle = (float(entry.get_text()))*(180/3.14159)
							else:
								angle = float(entry.get_text())

							response = set_servo_angle(i, angle)
							if response.Success:
								print "Set Module %d to %f degrees in enter!" % (i,angle)
				except rospy.ServiceException, e:
					print "Service call failed: %s" % e
	
	# This is the callback function for if the angle dropdown menu option is changed.	
	def angle_change(self, widget):
		global angleUnit
			
		tempString = widget.get_active_text()

		if len(textBoxList) >= (numModules+1):
			for i in range(1,numModules+1):
				# Force an uneditable (activated) box to change units.
				if textBoxList[i].get_editable():
					angle = float(textBoxList[i].get_text())

					if cmp(tempString,"radians") == 0:
						# We only use two units, but for completeness, we
						# check what the previous unit was before conversion.
						if cmp(angleUnit,"degrees") == 0:
							angle = angle*(3.14159/180)
					elif cmp(tempString,"degrees") == 0:
						# We only use two units, but for completeness, we
						# check what the previous unit was before conversion.
						if cmp(angleUnit,"radians") == 0:
							angle = angle*(180/3.14159)

					textBoxList[i].set_text(str(angle))

			angleUnit = tempString

	# This is the callback function for if the distance unit menu option is changed.			
	def distance_change(self, widget):
		global numModules
		global textBoxList
		global distanceUnit
		global precision

		tempString = widget.get_active_text()

		if len(textBoxList) >= (numModules+4):
			for i in range(numModules+1, numModules+4):
				distance = float(textBoxList[i].get_text())
			
				# Figure out what we're converting to.
				if cmp(tempString,"feet") == 0:
					# Figure out what we're converting from.
					if cmp(distanceUnit,"inches") == 0:
						distance = distance/12
					elif cmp(distanceUnit,"meters") == 0:
						distance = distance*3.28
					elif cmp(distanceUnit,"centimeters") == 0:
						distance = distance*0.0328
					elif cmp(distanceUnit,"millimeters") == 0:
						distance = distance*0.00328
				elif cmp(tempString,"inches") == 0:
					# Figure out what we're converting from.
					if cmp(distanceUnit,"feet") == 0:
						distance = distance*12
					elif cmp(distanceUnit,"meters") == 0:
						distance = distance*39.37
					elif cmp(distanceUnit,"centimeters") == 0:
						distance = distance*0.3937
					elif cmp(distanceUnit,"millimeters") == 0:
						distance = distance*0.03937
				elif cmp(tempString,"meters") == 0:
					# Figure out what we're converting from.
					if cmp(distanceUnit,"feet") == 0:
						distance = distance*0.3048
					elif cmp(distanceUnit,"inches") == 0:
						distance = distance*0.0254
					elif cmp(distanceUnit,"centimeters") == 0:
						distance = distance/100
					elif cmp(distanceUnit,"millimeters") == 0:
						distance = distance/1000
				elif cmp(tempString,"centimeters") == 0:
					# Figure out what we're converting from.
					if cmp(distanceUnit,"feet") == 0:
						distance = distance*30.48
					elif cmp(distanceUnit,"inches") == 0:
						distance = distance*2.54
					elif cmp(distanceUnit,"meters") == 0:
						distance = distance*100
					elif cmp(distanceUnit,"millimeters") == 0:
						distance = distance/10
				elif cmp(tempString,"millimeters") == 0:
					# Figure out what we're converting from.
					if cmp(distanceUnit,"feet") == 0:
						distance = distance*304.8
					elif cmp(distanceUnit,"inches") == 0:
						distance = distance*25.4
					elif cmp(distanceUnit,"meters") == 0:
						distance = distance*1000
					elif cmp(distanceUnit,"centimeters") == 0:
						distance = distance*10
			
				distance = float(int(distance*(10**precision)))/(10**precision)
				textBoxList[i].set_text(str(distance))
			
			distanceUnit = tempString

	# This custom destroy function stops the thread function before it dies.
	def destroy(self, widget, data=None):
		self.stop()
		gtk.main_quit()
		
	# This function initializes the window and its values.
	def __init__(self, *args):
		threading.Thread.__init__(self)

		global numModules
		global window
		global buttonList
		global textBoxList
		
		# Store the total number of modules we were passed.
		numModules = args[0]
		
		# Initialize our window.
		window.connect("destroy", self.destroy)
		window.set_title("Arm Controller")
		window.set_resizable(True)
		vbox = gtk.VBox(False, 10)
		hbox = gtk.HBox(True, 0)
		window.add(hbox)
		hbox.pack_start(vbox, True, True, 0)
		window.set_border_width(5)
		
		# Create the frame for arm controls.
		frame = gtk.Frame("Arm Controls")
		armvbox = gtk.VBox(False, 0)
		speedhbox = gtk.HBox(False, 0)
		anglehbox = gtk.HBox(False, 0)
		distancehbox = gtk.HBox(False, 0)

		# Create the checkbox that lets us toggle all power on or off.
		togglePowerAll = gtk.CheckButton("Power All")
		togglePowerAll.connect("toggled", self.callback, "Power All")
		armvbox.pack_start(togglePowerAll, True, True, 5)
		buttonList.append(togglePowerAll)

		# Create the checkbox that lets a user specify how to change speed.
		toggleSpeedAll = gtk.CheckButton("Speed All")
		toggleSpeedAll.connect("toggled", self.callback, "Speed All")
		armvbox.pack_start(toggleSpeedAll, False, True, 0)
		# can't be added b/c of module power checks buttonList.append(toggleSpeedAll)

		# Create the speed slider.
		label = gtk.Label("Speed (%):")
		speedhbox.pack_start(label, False, True, 0)
		adjust1 = gtk.Adjustment(50.0,0.0,100.0,1.0,10.0,0.0)
		speedScale = gtk.HScale(adjust1)
		speedScale.set_digits(0)
		speedScale.connect("value-changed", self.callback, "Speed All")
		speedhbox.pack_start(speedScale, True, True, 0)
		armvbox.pack_start(speedhbox, False, True, 0)

		# Allow the user to specify the angle unit.
		label = gtk.Label("Angle Units:      ")
		anglehbox.pack_start(label, False, True, 0)
		unitBox = gtk.combo_box_new_text()
		unitBox.connect("changed", self.angle_change)
		unitBox.append_text("degrees")
		unitBox.append_text("radians")
		unitBox.set_active(0)
		anglehbox.pack_start(unitBox, True, True, 0)
		armvbox.pack_start(anglehbox, False, True, 0)

		# Allow the user to specify the distance unit.
		label = gtk.Label("Distance Units: ")
		distancehbox.pack_start(label, False, True, 0)
		unitBox = gtk.combo_box_new_text()
		unitBox.connect("changed", self.distance_change)
		unitBox.append_text("feet")
		unitBox.append_text("inches")
		unitBox.append_text("meters")
		unitBox.append_text("centimeters")
		unitBox.append_text("millimeters")
		unitBox.set_active(4)
		distancehbox.pack_start(unitBox, True, True, 0)
		armvbox.pack_start(distancehbox, False, True, 0)

		# Add a software reset button for the root arm.
		armReset = gtk.Button("Reset Arm")
		armReset.connect("clicked", self.callback, "Reset Arm")
		armvbox.pack_start(armReset, False, True, 0)

		frame.add(armvbox)
		vbox.pack_start(frame, False, True, 0)

		# Create a dummy text box that doesn't get displayed so that
		# our index range starts at 1.
		entry = gtk.Entry()
		textBoxList.append(entry)
		
		for i in range(1,numModules+1):
			# Create a frame for the module control panel.
			frame = gtk.Frame("Module " + str(i))

			# Create a vertical box for all module settings.
			modvbox = gtk.VBox(False, 0)
			# Create a horizontal box for the angle box.
			anglehbox = gtk.HBox(False, 0)
			# Create a horizontal box for the speed slider.
			speedhbox = gtk.HBox(False, 0)

			# Create a check button for the power and add it to the hbox.
			button = gtk.CheckButton("Power " + str(i))
			button.connect("toggled", self.callback, "Power " + str(i))
			modvbox.pack_start(button,True,True,2)
			buttonList.append(button)
			
			# Create a text box for the angle.
			label = gtk.Label("Angle: ")
			anglehbox.pack_start(label, False, True, 0)
			entry = gtk.Entry()
			entry.set_max_length(6)
			entry.set_width_chars(6)
			entry.connect("activate", self.enter_callback, entry)
			entry.set_text("0")
			entry.set_name("Angle " + str(i))			
			entry.set_editable(False)
			anglehbox.pack_start(entry, False, True, 0)
			textBoxList.append(entry)

			# Create a label and slider for the servo speed.
			label = gtk.Label("Speed (%):")
			speedhbox.pack_start(label, False, True, 0)
			adjust1 = gtk.Adjustment(50.0,0.0,100.0,1.0,10.0,0.0)
			speedScale = gtk.HScale(adjust1)
			speedScale.set_digits(0)
			speedScale.connect("value-changed", self.callback, "Speed " + str(i))
			speedhbox.pack_start(speedScale, True, True, 0)

			# Pack it all together.
			modvbox.pack_start(anglehbox, False, False, 0)
			modvbox.pack_start(speedhbox, False, False, 0)
			
			frame.add(modvbox)

			vbox.pack_start(frame, False, True, 0)
		
		# Create the XYZ interaction frame and boxes
		frame = gtk.Frame("Arm Tip Coordinates (XYZ)")
		xyzvbox = gtk.VBox(False, 5)

		# Create the horizontal box for the XYZ coordinate boxes.
		xyzhbox = gtk.HBox(True, 0)
		entry = gtk.Entry()
		entry.set_max_length(6)
		entry.set_width_chars(6)
		entry.connect("activate", self.enter_callback, entry)
		entry.set_text("0")
		entry.set_name("X")
		xyzhbox.pack_start(entry, False, True, 0)
		entry.set_editable(False)
		entry.show()
		textBoxList.append(entry)
		
		entry = gtk.Entry()
		entry.set_max_length(6)		
		entry.set_width_chars(6)
		entry.connect("activate", self.enter_callback, entry)
		entry.set_text("0")
		entry.set_name("Y")
		xyzhbox.pack_start(entry, False, True, 0)
		entry.set_editable(False)
		textBoxList.append(entry)
		
		entry = gtk.Entry()
		entry.set_max_length(6)
		entry.set_width_chars(6)
		entry.connect("activate", self.enter_callback, entry)
		entry.set_text("0")
		entry.set_name("Z")
		xyzhbox.pack_start(entry, False, True, 0)
		entry.set_editable(False)
		textBoxList.append(entry)
		xyzvbox.pack_start(xyzhbox, False, True, 0)
		
		# Add in an IK solver button.
		ikEntry = gtk.Button("Send Coordinates")
		ikEntry.connect("clicked", self.callback, "Send Coordinates")
		xyzvbox.pack_start(ikEntry, False, True, 0)
		buttonList.append(ikEntry)

		frame.add(xyzvbox)
		vbox.pack_start(frame, False, True, 0)

		# Pose and motion functionality.
		frame = gtk.Frame("File Management")
		posevbox = gtk.VBox(False, 0)
		mothbox = gtk.HBox(False, 0)
		entry = gtk.Entry()
		entry.set_max_length(20)
		entry.set_width_chars(20)
		entry.connect("activate", self.enter_callback, entry)
		entry.set_text("pose1")
		entry.set_name("POSE")
		posevbox.pack_start(entry, False, True, 0)
		entry.set_editable(True)
		textBoxList.append(entry)

		# The buttons are packed in a seperate horizontal box before being added
		# into the vertical box of the frame and finally packed into the window
		poseEntry = gtk.Button("Save Pose")
		poseEntry.connect("clicked", self.callback, "Save Pose")
		mothbox.pack_start(poseEntry, True, True, 0)
		buttonList.append(poseEntry)

		motRun = gtk.Button("Run Motion")
		motRun.connect("clicked", self.callback, "Run Motion")
		mothbox.pack_start(motRun, True, True, 0)
		buttonList.append(motRun)
		posevbox.pack_start(mothbox, False, False, 0)
		frame.add(posevbox)
		vbox.pack_start(frame, True, True, 0)

		vbox.show()
		window.show_all()
		self.start()

# The number of modules the master node found.
numModules = 0
# This array stores the button objects as they are created.
buttonList = []
# This array stores the text entry objects as they are created.
textBoxList = []
# This sets float precision.
precision = 2
# This variable allows the "All" box to be manipulated without
# changing every other box.
notAll = True
# Store the angle unit (starts as degrees).
angleUnit = "degrees"
# Store the distance unit (starts as millimeters).
distanceUnit = "millimeters"
# Our GUI window object.
window = gtk.Window(gtk.WINDOW_TOPLEVEL)

if __name__ == "__main__":
	# Wait until we find the number of modules (which also means comm is established).
	rospy.wait_for_service('get_module_total')
	try:
		get_module_total = rospy.ServiceProxy('get_module_total', GetModuleTotal)
		response = get_module_total(1)
		ModuleControllerGUI(response.Total)
	except rospy.ServiceException, e:
		print "Service call failed: %s" % e

	# GTK threads enter and leave have to wrap all GUI-altering functions or it will crash.
	gtk.gdk.threads_enter()
	gtk.main()
	gtk.gdk.threads_leave()
