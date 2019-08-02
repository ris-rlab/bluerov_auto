#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PointStamped, Point, Pose, Quaternion, Twist, Vector3

odom = Odometry()


def imu_callback(msg):
	global odom
	imu_x= msg.orientation.x
	imu_y= msg.orientation.y
	imu_z= msg.orientation.z
	imu_w= msg.orientation.w
	print imu_x, imu_y,imu_z,imu_w, 'imu'

	current_time = rospy.Time.now()

	imu = Imu()
	imu.header.stamp = current_time
	imu.header.frame_id = "odom"

	odom.pose.pose.orientation.x = imu_x
	odom.pose.pose.orientation.y = imu_y
	odom.pose.pose.orientation.z = imu_z
	odom.pose.pose.orientation.w = imu_w

	# publish the message
	#imu_pub.publish(odom)



def wl_callback(msg):
	global odom
	#print msg.point
	x=msg.point.x
	y=msg.point.y
	z=msg.point.z
	print x,y,z,'waterlinked'

	#odom_broadcaster = tf.TransformBroadcaster()
	current_time = rospy.Time.now()
	# next, we'll publish the odometry message over ROS
	odom.header.stamp = current_time
	odom.header.frame_id = "odom"

	# set the position

	odom.pose.pose.position.x = x
	odom.pose.pose.position.y = y
	odom.pose.pose.position.z = z

	odom_pub.publish(odom)


rospy.init_node("check_odometry")

wl_sub=rospy.Subscriber("/waterlinked/acoustic_position/raw",PointStamped,wl_callback)
imu_sub=rospy.Subscriber('/mavros/imu/data',Imu,imu_callback)
imu_pub=rospy.Publisher("/odom",Odometry,queue_size=50)
odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)

rospy.spin()
