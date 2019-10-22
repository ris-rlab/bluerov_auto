#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf import TransformBroadcaster
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PointStamped, Point, Pose, Twist, Vector3, TwistStamped, TwistWithCovarianceStamped
from math import sin, cos, radians

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

    odom.pose.pose.orientation.x = -imu_y
    odom.pose.pose.orientation.y = imu_x
    odom.pose.pose.orientation.z = imu_z
    odom.pose.pose.orientation.w = imu_w
	
    odom.pose.covariance = [0.2, 0, 0, 0, 0, 0, 
                            0, 0.2, 0, 0, 0, 0, 
                            0, 0, 0.2, 0, 0, 0, 
                            0, 0, 0, 0.2, 0, 0, 
                            0, 0, 0, 0, 0.2, 0, 
                            0, 0, 0, 0, 0, 0.2]

def wl_callback(msg):
    global odom
    
    #print msg.point
    x = msg.point.x
    y = msg.point.y
    z = msg.point.z
    print x, y, z, 'waterlinked'
    
    # rotationmatrix
    a = radians(90)
    x1 = x * cos(-a) - y * sin(-a)
    y1 = x * sin(-a) + y * cos(-a)

    # next, we'll publish the odometry message over ROS
    current_time = rospy.Time.now()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"
    odom.child_frame_id = "base_link"

    # set the position
    odom.pose.pose.position.x = x1
    odom.pose.pose.position.y = y1
    odom.pose.pose.position.z = -z

    # set the orientation
    tf_orient_x = odom.pose.pose.orientation.x
    tf_orient_y = odom.pose.pose.orientation.y
    tf_orient_z = odom.pose.pose.orientation.z
    tf_orient_w = odom.pose.pose.orientation.w

    odom_pub.publish(odom)
    tf_pub.sendTransform((x1, y1, z), 
                         (tf_orient_x, tf_orient_y, tf_orient_z, tf_orient_w), 
                         current_time, 
                         "base_link", 
                         "odom")

def twist_callback(msg):

    global twist

    vx = msg.twist.linear.y
    vy = msg.twist.linear.x
    vz = -msg.twist.linear.z
    print vx, vy, vz, 'twist'

    current_time = rospy.Time.now()

    twist = TwistWithCovarianceStamped()
    twist.header.stamp = current_time
    twist.header.frame_id = "base_link"

    twist.twist.twist.linear.x = vx
    twist.twist.twist.linear.y = vy
    twist.twist.twist.linear.z = vz

    twist.twist.covariance = [0.2, 0, 0, 0, 0, 0, 
                              0, 0.2, 0, 0, 0, 0, 
                              0, 0, 0.2, 0, 0, 0, 
                              0, 0, 0, 0.2, 0, 0, 
                              0, 0, 0, 0, 0.2, 0, 
                              0, 0, 0, 0, 0, 0.2]
    
    twist_pub.publish(twist)

rospy.init_node("pub_odometry")

rospy.Subscriber("/waterlinked/acoustic_position/raw", PointStamped, wl_callback)
rospy.Subscriber('/mavros/imu/data', Imu, imu_callback)
rospy.Subscriber('/mavros/local_position/velocity_body', TwistStamped, twist_callback)

odom_pub = rospy.Publisher("/odom", Odometry, queue_size = 50)
twist_pub = rospy.Publisher("/twist", TwistWithCovarianceStamped, queue_size = 50)
tf_pub = TransformBroadcaster()

rospy.spin()
