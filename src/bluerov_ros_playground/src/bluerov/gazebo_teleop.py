#!/usr/bin/env python

import rospy
import time
import math as m
try:
    import pubs
    import subs
except:
    import bluerov.pubs as pubs
    import bluerov.subs as subs

from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Joy
from geometry_msgs.msg import Vector3 

def Angle(des, curr):
	x = 3.14-abs(des)+3.14-abs(curr)
	if abs(des-curr)>x:
		print("X is {}".format(x))
		if(des-curr)>0:
			return -x
		else:
			return x
	else:
		return des-curr

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
"""def(er):
	if(abs(er>20)):
		er=20 * er/abs(er)
	return er"""		
class GazeboTeleop(object):

    """Class to handle with gazebo teleop

    Attributes:
        pub (TYPE): ROS publisher
        sub (TYPE): ROS subscriber
    """

    def __init__(self):
        super(GazeboTeleop, self).__init__()

        self.sub = subs.Subs()
        self.pub = pubs.Pubs()
	self.pub1 = pubs.Pubs()
	self.sub1 = subs.Subs()
        self.pub.subscribe_topic('/BlueRov2/thruster_command', JointState)
	self.pub1.subscribe_topic('/gazebo/current',Vector3)
        self.sub.subscribe_topic('/BlueRov2/state', Odometry)
	#self.sub1.subscribe_topic('/BlueRov2/state', Odometry)
        #self.sub.subscribe_topic('/joy', Joy)

    def run(self):
	time.sleep(0.1)
        """ Run Gazebo Teleop
        """
	PI = 3.14
	P =[0 for u in range(3)]
	I = [0 for u in range(3)]
	D = [0 for u in range(3)]
	Pr = [0 for u in range(2)]
	Dr = [0 for u in range(2)]
	Ir =[0 for u in range(2)]
	er = [0 for u in range(2)]
	k_p = 370 
	k_i = 0.7
	k_d = 600
	timex=0.0
	k_py = 590
	k_dy = 150
	k_py2 = 100
	k_dy2 = 2.10
	rate = rospy.Rate(10)
	d = [14, -20]
	dy = 14
	old = [0 for u in range(2)]
	oldr = [0 for u in range(2)]
	oldy = 0
	oldy2 = 0
	oldy1 = 0
	check = 1
	dr = [0, 2.14]
	old_p = [0 for u in range(2)]
	if(d[0]>=0 and dy >=0):
		val = -PI/2 - PI/4
	elif (d[0]<0 and dy>0):
		val = PI/4
		check = 0
	elif(d[0]<0 and dy<0):
		val = -PI/4
	else:
		val = -PI/2 - PI/4
		check = 0
        while not rospy.is_shutdown():
      	    time.sleep(0.1)
            # Try to get data
	    
            try:
                # Get ROV position
		
                rospy.loginfo(self.sub.get_data()['BlueRov2']['state']['pose']['pose'])
            except Exception as error:
                print('Get data error:', error)

            try:
		x = self.sub.get_data()['BlueRov2']['state']['pose']['pose']['position']['x']
		y = self.sub.get_data()['BlueRov2']['state']['pose']['pose']['position']['y']
		z = self.sub.get_data()['BlueRov2']['state']['pose']['pose']['position']['z']
		#print("These are {} {} {}" .format(x,y,z))
		pos = [x,z]
		Py1 = -((dy-y)-(d[0]-x))*k_py2
		Dy1 = (-oldy1-((dy-y)-(d[0]-x)))*k_dy2
                ey1 = (Py1 + Dy1)/100
		oldy1 = -(dy-y)+(d[0]-x)
		q = self.sub.get_data()['BlueRov2']['state']['pose']['pose']['orientation']
    		sinr_cosp = +2.0 * (q['w'] * q['x'] + q['y'] * q['z'])
    		cosr_cosp = +1.0 - 2.0 * (q['x'] * q['x'] + q['y'] * q['y'])
    		sinp = +2.0 * (q['w'] * q['y'] - q['z'] * q['x'])
    		siny_cosp = +2.0 * (q['w'] * q['z'] + q['x'] * q['y'])
    		cosy_cosp = +1.0 - 2.0 * (q['y'] * q['y'] + q['z'] * q['z'])  
    		yaw = m.atan2(siny_cosp, cosy_cosp)
		roll = m.atan2(sinr_cosp, cosr_cosp)
		pitch = m.asin(sinp)
		posr = [yaw, pitch]
		if(ey1>PI/4):
			ey1 = PI/4
		elif (ey1<-PI/4):
			ey1 =-PI/4
		if(check==0):
			ey1=-ey1		
		dy2 = ey1+val
		v = 0#Angle(dy2, yaw)
		y1 = v*k_py + ((-oldy2 + v)/0.11)*k_dy 
		oldy2 = v
		print("Roll Pitch Yaw: {} {} {}".format(roll, pitch, yaw))	
                # Get joystick data and send it to Gazebo model
                joy = self.sub.get_data()['joy']['axes']
		e = [0 for u in range(2)]
		
                forces = [0 for u in range(6)]
		for i in range(2):
			P[i] = (d[i]-pos[i])*k_p
			I[i] += (d[i]-pos[i])*k_i*0.11
			D[i] = ((-old[i]+d[i]-pos[i])/0.11)*k_d
			#print("Initial Error: {}".format(old[i]))
			#print("Current Error: {}".format(d[i]-pos[i]))
			#print("Rate: {}".format(pos[i]-old_p[i]))
			if I[i]>5 or I[i]<-5:
				I[i]=0
			e[i] = P[i]+I[i]+D[i]
			old[i] = d[i]-pos[i]
			old_p[i] = pos[i]
		for i in range(2):
			val1 = Angle(dr[i], posr[i])
			Pr[i] = val1*k_py
			Dr[i] = -(abs(Val(oldr[i], val1, dr[i]))/0.11)*k_dy*(Pr[i]/abs(Pr[i]))
                        print("D[i] P[i] I[i] {} {}".format(Dr[i], Pr[i]))
			er[i] = Pr[i]+Dr[i]
			oldr[i] = val1
		if(abs(er[0])>20):
			er[0]=20 * er[0]/abs(er[0])
		#	
                # joystick analog enums
                #  left stick: 1 = up down; 0 = right left
                #  right stick: 4 = up down; 3 = right left
		#y1-yaw er[0]-pitch er[1]-roll
                #forces[0] = er[0]/4#y1/4#e[0]/4###+# + # 
                #forces[1] = -er[0]/4#-y1/4#e[0]/4### # #
                #forces[2] = -er[0]/4#-y1/4#-e[0]/4#### + # 
                #forces[3] = er[0]/4#y1/4#-e[0]/4###+#-e[0]/4 # 
                #forces[4] = 0#e[1]/2#er[1]/2#0## #+ 
                #forces[5] = 0#e[1]/2#-er[1]/2#0# #-
		print("{}".format(er[0])) 
		print("{} {} {}".format(timex, d[0]-pos[0], pos[0]))
		forces[0] = joy[1] +joy[0]
                forces[1] = joy[1] -joy[0]
                forces[2] = -joy[1] -joy[0]
                forces[3] = -joy[1] +joy[0]
                forces[4] = joy[4] +joy[3]
                forces[5] = joy[4] -joy[3]
		vel = Vector3(0, 0.1, 0)
                joint = JointState()
                joint.name = ["thr{}".format(u + 1) for u in range(6)]
                joint.position = [pwm for pwm in forces]
		#print("This is {} {} {} {} {} {}".format(joint.position[0], joint.position[1],joint.position[2], joint.position[3], joint.position[4], joint.position[5]))
		#print("This is {} {} {} {} {} {}".format(forces[0], forces[1],forces[2], forces[3], forces[4], forces[5]))
		
                self.pub.set_data('/BlueRov2/thruster_command', joint)
		self.pub1.set_data('/gazebo/current', vel)
            except Exception as error:
                print('rc error:', error)
	    

if __name__ == "__main__":
    try:
        rospy.init_node('gazebo_teleop', log_level=rospy.DEBUG)
    except rospy.ROSInterruptException as error:
        print('pubs error with ROS: ', error)
        exit(1)
    gazebo_teleop = GazeboTeleop()
    gazebo_teleop.run()
