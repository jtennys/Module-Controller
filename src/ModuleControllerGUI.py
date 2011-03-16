#!/usr/bin/env python
import roslib; roslib.load_manifest('module_controller')
from module_controller.srv import *
import rospy
import threading
import time

import pygtk
pygtk.require('2.0')
import gtk

# Initialize the gtk thread engine.
gtk.gdk.threads_init()

class ModuleControllerGUI(threading.Thread):
	# Create the killthread event for stopping the thread.
	killthread = threading.Event()

	def run(self):
		global buttonList
		global textBoxList
		global numModules
		global angleUnit
		
		while not (self.killthread.isSet() or rospy.is_shutdown()):
			for i in range(1,numModules+1):
				# Don't print to an active servo box.  This is done
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

	def stop(self):
		self.killthread.set()
		
	# Function that is called when an click action happens.
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
				elif widget.get_active() == False:
					# If "All" has been clicked off, uncheck everything.
					for i in range(1,numModules+1):
						buttonList[i].set_active(False)

		elif cmp(data, "Send Coordinates") == 0:
			if buttonList[0].get_active():
				print textBoxList[numModules+1].get_text()
				print textBoxList[numModules+2].get_text()
				print textBoxList[numModules+3].get_text()
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
			else:
				# Create a variable to check if all modules are on.
				# If they are, we will set the "All" box to true.
				allOn = True
				for i in range(1,numModules+1):
					if buttonList[i].get_active() == False:
						allOn = False
				
				if allOn:
					buttonList[0].set_active(True)
				
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
			if (cmp(widget.get_name(),"X") != 0) and \
			   (cmp(widget.get_name(),"Y") != 0) and \
			   (cmp(widget.get_name(),"Z") != 0):
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
				except rospy.ServiceException, e:
					print "Service call failed: %s" % e
		
	def angle_change(self, widget):
		global angleUnit
			
		tempString = widget.get_active_text()

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
			
	def distance_change(self, widget):
		global numModules
		global textBoxList
		global distanceUnit
		global precision

		tempString = widget.get_active_text()
		
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

	def destroy(self, widget, data=None):
		self.stop()
		gtk.main_quit()
		

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
		window.set_title("Module Controller")
		window.set_resizable(False)
		vbox = gtk.VBox(False, 5)
		hbox = gtk.HBox(False, 5)
		window.add(hbox)
		hbox.pack_start(vbox, False, False, 0)
		window.set_border_width(5)
		
		# Create the checkbox that lets us toggle all power on or off.
		togglePowerAll = gtk.CheckButton("Power All")
		togglePowerAll.connect("toggled", self.callback, "Power All")
		vbox.pack_start(togglePowerAll, True, True, 2)
		togglePowerAll.show()
		buttonList.append(togglePowerAll)

		# Create a dummy text box that doesn't get displayed so that
		# our index range starts at 1.
		entry = gtk.Entry()
		textBoxList.append(entry)
		
		for i in range(1,numModules+1):
			modPowerLabel = "Module " + str(i) + " Power"
			modAngleLabel = "Module " + str(i) + " Angle"
			
			button = gtk.CheckButton(modPowerLabel)
			button.connect("toggled", self.callback, modPowerLabel)
			vbox.pack_start(button,True,True,2)
			button.show()
			buttonList.append(button)
			
			entry = gtk.Entry()
			entry.set_max_length(6)
			entry.connect("activate", self.enter_callback, entry)
			entry.set_text("0")
			entry.set_name(modAngleLabel)
			vbox.pack_start(entry, True, True, 0)
			entry.set_editable(False)
			entry.show()
			textBoxList.append(entry)
		
		# Allow the user to specify the angle unit.
		frame = gtk.Frame("Angle Units")
		unitBox = gtk.combo_box_new_text()
		unitBox.connect("changed", self.angle_change)
		unitBox.append_text("degrees")
		unitBox.append_text("radians")
		unitBox.set_active(0)
		frame.add(unitBox)
		vbox.pack_start(frame, True, True, 0)
		
		# Create the x y z text entry fields
		entry = gtk.Entry()
		entry.set_max_length(8)
		entry.connect("activate", self.enter_callback, entry)
		entry.set_text("0")
		entry.set_name("X")
		vbox.pack_start(entry, True, True, 0)
		entry.set_editable(False)
		entry.show()
		textBoxList.append(entry)
		
		entry = gtk.Entry()
		entry.set_max_length(8)
		entry.connect("activate", self.enter_callback, entry)
		entry.set_text("0")
		entry.set_name("Y")
		vbox.pack_start(entry, True, True, 0)
		entry.set_editable(False)
		entry.show()
		textBoxList.append(entry)
		
		entry = gtk.Entry()
		entry.set_max_length(8)
		entry.connect("activate", self.enter_callback, entry)
		entry.set_text("0")
		entry.set_name("Z")
		vbox.pack_start(entry, True, True, 0)
		entry.set_editable(False)
		entry.show()
		textBoxList.append(entry)

		# Allow the user to specify the distance unit.
		frame = gtk.Frame("Distance Units")
		unitBox = gtk.combo_box_new_text()
		unitBox.connect("changed", self.distance_change)
		unitBox.append_text("feet")
		unitBox.append_text("inches")
		unitBox.append_text("meters")
		unitBox.append_text("centimeters")
		unitBox.append_text("millimeters")
		unitBox.set_active(1)
		frame.add(unitBox)
		vbox.pack_start(frame, True, True, 0)
		
		ikEntry = gtk.Button("Send Coordinates")
		ikEntry.connect("clicked", self.callback, "Send Coordinates")
		vbox.pack_start(ikEntry, True, True, 2)
		ikEntry.show()
		
		vbox.show()
		window.show_all()
		self.start()

# The number of modules the master node found.
# This is just a placeholder until I find out how to pass this in.
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

# Store the angle unit (starts as degrees)
angleUnit = "degrees"
	
# Store the distance unit (starts as inches)
distanceUnit = "inches"

window = gtk.Window(gtk.WINDOW_TOPLEVEL)

if __name__ == "__main__":
	modGUI = ModuleControllerGUI(3)
	gtk.gdk.threads_enter()
	gtk.main()
	gtk.gdk.threads_leave()
