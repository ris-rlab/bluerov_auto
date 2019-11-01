import rospy
import time
import math as m
try:
    import pubs
    import subs
except:
    import bluerov.pubs as pubs
    import bluerov.subs as subs
from mavros_msgs.msg import OverrideRCIn
from bluerov_ros_playground.msg import Custom
from sensor_msgs.msg import Joy 
class Control(object):
    def __init__(self):
        super(Control, self).__init__()

        # Do what is necessary to start the process
        # and to leave gloriously

        self.sub = subs.Subs()
        self.pub = pubs.Pubs()
    	self.sub.subscribe_topic('/actuation', Custom)
	self.sub.subscribe_topic('/act', Custom)
	self.sub.subscribe_topic('/joy', Joy)
        self.pub.subscribe_topic('/mavros/rc/override', OverrideRCIn)
    def run(self):
	au_state = True;
        """Sends control input to the robot by setting appropriate levels of PWM
           in the /mavros/rc/override
           Inputs: -1.0...1.0 for each of the force and torque axis
                    (it corresponds to the desired velocity level)
                    (the unit is arbitrary)
                    camupdown is in the same range, deciding if the camera should be
                    pitched a step down or a step up
        """
	state = False
	state_a = False
	while not rospy.is_shutdown():
		time.sleep(0.1)	
		override = [1500,1500,1500,1500,1500,1500,1500,1500]
		print("ABCDEFGHIJKLMNOPQRSTUV")

		"""if(self.sub.get_data()['Custom']['id']=="joystick"):
			a = self.sub.get_data()['Custom']['data']
			au_state = False
		elif(self.sub.get_data()['Custom']['id']=="autonomous"):
			if(au_state == True):
				a = self.sub.get_data()['Custom']['data']"""
			
		try:
			but = self.sub.get_data()['joy']['buttons']	
			if(but[0]==1):
				state = not state
			print("ABCDEFGHIJKLMNOPQRSTUV")
			if(state !=state_a):
				if(self.sub.get_data()['actuation']['id']=="autonomous"):
					a = self.sub.get_data()['actuation']['data']
			else:
				if(self.sub.get_data()['act']['id']=="joystick"):
					a = self.sub.get_data()['act']['data']
			#print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")	
		# rc run between 1100 and 2000, a joy command is between -1.0 and 1.0
		# Correction SK:  1100 and 1900
		#override = [int(val*400 + 1500) for val in joy]
		#for _ in range(len(override), 8):
		#    override.append(0)
		
		# Not implemented in MAVLink apparently:
		#override = [1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500]
		# Surge = Channel 5
			override = [1500,1500,1500,1500,1500,1500,1500,1500]
			override[4] = int(a[0]*400 + 1500)
			# Sway = Channel 6
			override[5] = int(a[1]*400 + 1500)
			# Heave = Channel 3
			override[2] = int(a[2]*400 + 1500)
			# Roll = Channel 2
			override[1] = int(a[3]*400 + 1500)
			# Pitch = Channel 1
			override[0] = int(a[4]*400 + 1500)
			# Yaw = Channel 4
			override[3] = int(a[5]*400 + 1500)
			# Cam up = Channel 8
			#override[7] = int(a[6]*400 + 1500)

			# Unused:
			# Ch 1 = Pitch
			# Ch 2 = Roll
			# Ch 7 = ?
			# Not implemented in MAVLink apparently:
			# Lights 1 & 2
			#override[8] = int(joy[6]*400 + 1500)
			#override[9] = int(joy[6]*400 + 1500)
			# Send joystick data as rc output into rc override topic
			# (fake radio controller)
			self.pub.set_data('/mavros/rc/override', override)
	        except Exception as error:
			print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")	
                	print('joy error:', error)
        
if __name__ == "__main__":
    try:
        rospy.init_node('control_node', log_level=rospy.DEBUG)
    except rospy.ROSInterruptException as error:
        print('pubs error with ROS: ', error)
        exit(1)
    code = Control()
    code.run()
