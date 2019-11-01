#!/usr/bin/env python

import cv2
import rospy
import time
import math as m
try:
    import pubs
    import subs
    import video
except:
    import bluerov.pubs as pubs
    import bluerov.subs as subs
    import bluerov.video as video

from geometry_msgs.msg import TwistStamped
from mavros_msgs.srv import CommandBool
from sensor_msgs.msg import JointState, Joy, Imu
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, FluidPressure
from mavros_msgs.msg import OverrideRCIn, RCIn, RCOut
from std_msgs.msg import Float32
from bluerov_ros_playground.msg import Custom
def Angle(des, curr):
	x = 3.14-abs(des)+3.14-abs(curr)
	if abs(des-curr)>x:
		print("X is {}".format(x))
		if(des-curr)>0:
			return x
		else:
			return -x
	else:
		return curr-des

def Val(old, curr, des):
	if((old>0 and curr<0) or (old<0 and curr>0)):
				if(abs(old-des)>2):
					x = 3.14-abs(old-des)+ 3.14-abs(curr-des)
					print("GIVE IT")
					return x
				else:
					return curr-old
	else:

		return curr - old
def error(er):
	if(abs(er>20)):
		er=20 * er/abs(er)
	return er

class Code(object):

    """Class to provide user access

    Attributes:
        cam (Video): Video object, get video stream
        pub (Pub): Pub object, do topics publication
        sub (Sub): Sub object, subscribe in topics
    """

    curr_pitch_setting = 0.0
    curr_roll_setting = 0.0 

    def enforce_limit(self, value):
        if (value < -1.0):
            value = -1.0
        elif (value > 1.0):
            value = 1.0
        return value

    def __init__(self):
        super(Code, self).__init__()

        # Do what is necessary to start the process
        # and to leave gloriously
        self.arm()

        self.sub = subs.Subs()
        self.pub = pubs.Pubs()
	self.pub.subscribe_topic('/actuation', Custom)
        self.pub.subscribe_topic('/mavros/rc/override', OverrideRCIn)
        self.pub.subscribe_topic('/mavros/setpoint_velocity/cmd_vel', TwistStamped)
        self.pub.subscribe_topic('/BlueRov2/body_command', JointState)
	self.pub.subscribe_topic('/Location2', Odometry)
        self.sub.subscribe_topic('/joy', Joy)
	self.sub.subscribe_topic('/odom', Odometry)
        self.sub.subscribe_topic('/mavros/battery', BatteryState)
        self.sub.subscribe_topic('/mavros/rc/in', RCIn)
        self.sub.subscribe_topic('/mavros/rc/out', RCOut)
        self.sub.subscribe_topic('/mavros/imu/static_pressure', FluidPressure)
        self.sub.subscribe_topic('/mavros/imu/diff_pressure', FluidPressure)
	self.sub.subscribe_topic('/mavros/imu/data', Imu)
	self.sub.subscribe_topic('/depth', Float32)
	self.sub.subscribe_topic('/odometry/filtered', Odometry)
        self.cam = None
        try:
            video_udp_port = rospy.get_param("/user_node/video_udp_port")
            rospy.loginfo("video_udp_port: {}".format(video_udp_port))
            self.cam = video.Video(video_udp_port)
        except Exception as error:
            rospy.loginfo(error)
            self.cam = video.Video()


    def arm(self):
        """ Arm the vehicle and trigger the disarm
        """
        rospy.wait_for_service('/mavros/cmd/arming')

        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.arm_service(True)

        # Disarm is necessary when shutting down
        rospy.on_shutdown(self.disarm)


    @staticmethod
    def pwm_to_thrust(pwm):
        """Transform pwm to thruster value
        The equation come from:
            https://colab.research.google.com/notebook#fileId=1CEDW9ONTJ8Aik-HVsqck8Y_EcHYLg0zK

        Args:
            pwm (int): pwm value

        Returns:
            float: Thrust value
        """
        return -3.04338931856672e-13*pwm**5 \
            + 2.27813523978448e-9*pwm**4 \
            - 6.73710647138884e-6*pwm**3 \
            + 0.00983670053385902*pwm**2 \
            - 7.08023833982539*pwm \
            + 2003.55692021905

   
    def run(self):
        """Run user code
        """
	dr = [0.0, 1.74]
	k_py = 250
	k_dy = 50
	k_iy = 1
	Iz = 0
	Iz1 = 0
	
	oldr = [0 for u in range(2)]
	Pr = [0 for u in range(2)]
	Dr = [0 for u in range(2)]
	Ir = [0 for u in range(2)]
	er = [0 for u in range(2)]
	P =[0 for u in range(3)]
	I = [0 for u in range(3)]
	D = [0 for u in range(3)]
	k_p = [390, 300, 170] 
	k_i = [20, 20, 50]
	k_d = [690, 590, 500]
	boolx = True
	old = [0 for u in range(3)]
	e = [0 for u in range(3)]
	d = [-4, 3, -1]
	xtra = 0
	rospy.wait_for_service('/mavros/cmd/arming')
        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.arm_service(True)
        while not rospy.is_shutdown():
            time.sleep(0.1)
            # Try to get data
            try:
                rospy.loginfo("Battery voltage: %f"%self.sub.get_data()['mavros']['battery']['voltage'])
                rospy.loginfo("Static pressure: %f"%self.sub.get_data()['mavros']['imu']['static_pressure']['fluid_pressure'])
                rospy.loginfo("Diff pressure: %f"%self.sub.get_data()['mavros']['imu']['diff_pressure']['fluid_pressure'])
                #rospy.loginfo(self.sub.get_field('fluid_pressure')['mavros']['imu']['static_pressure'])
                #rospy.loginfo(self.sub.get_field('fluid_pressure')['mavros']['imu']['diff_pressure'])
                #rospy.loginfo(self.sub.get_data()['mavros']['rc']['in']['channels'])
                #rospy.loginfo(self.sub.get_data()['mavros']['rc']['out']['channels'])
            except Exception as error:
                print('Get data error:', error)

            try:
                # Get joystick data
                joy = self.sub.get_data()['joy']['axes']
                but = self.sub.get_data()['joy']['buttons']
		depth = self.sub.get_data()['depth']['data']
		# Converting button presses to roll and pitch setting modification
		self.curr_pitch_setting = self.enforce_limit(self.curr_pitch_setting - joy[7]*0.1)
		self.curr_roll_setting = self.enforce_limit(self.curr_roll_setting - joy[6]*0.1)
		if (but[4] == 1):
		    self.curr_pitch_setting = 0.0
		if (but[5] == 1):
		    self.curr_roll_setting = 0.0                    
		if(boolx):		
			rospy.wait_for_service('/mavros/cmd/arming')
       			arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        		arm_service(True)
			boolx = False
		
		forces = [0.0, 0.0, 0.0]
		torques = [0.0, 0.0, 0.0]
		pos1 = self.sub.get_data()['odom']['pose']['pose']['position']
		x1 = pos1['x']
		y1 = pos1['y']
		print("Next")
		pos = [x1, y1, depth]
		a = self.sub.get_data()['mavros']['imu']['data']
		acc = a['linear_acceleration']
		print("x: {}  y: {} z: {} ".format(acc['x'], acc['y'], acc['z']))
		print("True Height: {}".format(Iz1-0.25))
		q = a['orientation']
		print("ROll YAW:")
		sinr_cosp = +2.0 * (q['w'] * q['x'] + q['y'] * q['z'])
    		cosr_cosp = +1.0 - 2.0 * (q['x'] * q['x'] + q['y'] * q['y'])
    		sinp = +2.0 * (q['w'] * q['y'] - q['z'] * q['x'])
    		siny_cosp = +2.0 * (q['w'] * q['z'] + q['x'] * q['y'])
    		cosy_cosp = +1.0 - 2.0 * (q['y'] * q['y'] + q['z'] * q['z'])  
    		yaw = m.atan2(siny_cosp, cosy_cosp)
		roll = m.atan2(sinr_cosp, cosr_cosp)
		posr = [yaw, roll]
		print("YAW ROll : {} {}".format(posr[0], posr[1]))
		for i in range(2):
			val1 = Angle(dr[i], posr[i])
			Pr[i] = val1*k_py
			#print("ROll YAW: {} {}".format(posr[0], posr[1]))
			Dr[i] = -(abs(Val(oldr[i], val1, dr[i]))/0.11)*k_dy*(Pr[i]/abs(Pr[i]))
                        #print("D[i] P[i] I[i] {} {}".format(Dr[i], Pr[i]))
			Ir[i] += val1*k_iy*0.11
			er[i] = Pr[i]+Dr[i]+Ir[i]
			oldr[i] = val1
		for i in range(3):
			print("Error101 {}".format(d[i]-pos[i]))
			P[i] = (d[i]-pos[i])*k_p[i]
			I[i] += (d[i]-pos[i])*k_i[i]*0.11
			D[i] = ((-old[i]+d[i]-pos[i])/0.11)*k_d[i]
			#print("Initial Error: {}".format(old[i]))
			#print("Current Error: {}".format(d[i]-pos[i]))
			#print("Rate: {}".format(pos[i]-old_p[i]))
			if I[i]>150 or I[i]<-150:
				I[i]=150 * I[i]/abs(I[i])
			e[i] = P[i]+I[i]+D[i]
			old[i] = d[i]-pos[i]
		if(abs(e[2])>1000):
			e[2]=1000 * e[2]/abs(e[2])
		if(abs(er[0])>650):
			er[0]=650 * er[0]/abs(er[0])
		print("{} {} {} is positon".format(pos[0], pos[1], pos[2]))
		if(abs(e[0])>1000):
			e[0]=1000 * e[0]/abs(e[0])
		if(abs(e[1])>900):
			e[1]=900 * e[1]/abs(e[1])
		print("Error is: {} {}".format(er[0], e[1]))
		"""if(xtra<20):		
			torques[2] = er[0]/845
			forces[0] = 0#-e[0]/(1300)#joy[4]/1.5#e[0]/(1300) #Surge
			xtra = xtra+1
		if(xtra>=20 and xtra<50):
			torques[2] = 0
			forces[0] = -e[0]/(1300)
			xtra = xtra+1
		if(xtra>=50):
			xtra = 0	
		print("XXXXTRA  {}".format(xtra))"""
		torques[2] = er[0]/950
		forces[0] = e[1]/(1300)*(1-er[0]/950)
		forces[1] = (1-er[0]/950)*e[0]/1300#-joy[3]/1.50##-joy[3]/1.5#e[1]/400 #Sway
		forces[2] = e[2]/300#joy[1]/1.5#joy[1]/1.50#joy[1]/1.50#e[2]/300 #Heave
		torques[0] = self.curr_roll_setting #Roll
		torques[1] = self.curr_pitch_setting #Pitch
		#torques[2] = -joy[0]/1.5 #Yaw
		data = Custom()
		data.id = "autonomous"
		data.data = forces + torques
		self.pub.set_data('/actuation', data)

            except Exception as error:
                print('joy error:', error)

            try:
                # Get pwm output and send it to Gazebo model
		rc = self.sub.get_data()['mavros']['rc']['out']['channels']
                
                joint = JointState()
                joint.name = ["thr{}".format(u + 1) for u in range(5)]
                joint.position = [self.pwm_to_thrust(pwm) for pwm in rc]
		
                self.pub.set_data('/BlueRov2/body_command', joint)
            except Exception as error:
                print('rc error:', error)

            try:
                if not self.cam.frame_available():
                    continue

                # Show video output
                frame = self.cam.frame()
                # Added code SK
                height, width, depth = frame.shape
                newframe = cv2.resize(frame,(int(width*0.65),int(height*0.65)))
                cv2.imshow('frame', newframe)
                #cv2.imshow('frame', frame)
                cv2.waitKey(1)
            except Exception as error:
                print('imshow error:', error)

    def disarm(self):
        self.arm_service(False)


if __name__ == "__main__":
    try:
        rospy.init_node('aut_node', log_level=rospy.DEBUG)
    except rospy.ROSInterruptException as error:
        print('pubs error with ROS: ', error)
        exit(1)
    code = Code()
    code.run()

