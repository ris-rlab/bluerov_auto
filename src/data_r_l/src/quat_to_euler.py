#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

def get_rotation (msg):
    
    global roll, pitch, yaw
    
    orientation_q = msg.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    print 'yaw', yaw, 'roll', roll, 'pitch', pitch

    current_time = rospy.Time.now()

    imu = Imu()
    imu.header.stamp = current_time
    imu.header.frame_id = "base_link"

    imu.orientation.x = -msg.orientation.y #pitch
    imu.orientation.y = msg.orientation.x #roll
    imu.orientation.z = msg.orientation.z #yaw
    imu.orientation.w = msg.orientation.w
    imu.orientation_covariance = [0.01, 0, 0, 
                                  0, 0.01, 0, 
                                  0, 0, 0.01]

    imu.angular_velocity.x = -msg.angular_velocity.y
    imu.angular_velocity.y = msg.angular_velocity.x
    imu.angular_velocity.z = msg.angular_velocity.z
    imu.angular_velocity_covariance = [0.01, 0, 0, 
                                       0, 0.01, 0, 
                                       0, 0, 0.01]

    imu.linear_acceleration.x = -msg.linear_acceleration.y
    imu.linear_acceleration.y = msg.linear_acceleration.x
    imu.linear_acceleration.z = msg.linear_acceleration.z
    imu.linear_acceleration_covariance = [0.01, 0, 0, 
                                          0, 0.01, 0, 
                                          0, 0, 0.01]

    euler_pub.publish(imu)

rospy.init_node('quaternion_to_euler')

rospy.Subscriber ('/mavros/imu/data', Imu, get_rotation)

euler_pub = rospy.Publisher("/imu_quat", Imu, queue_size = 50)

rospy.spin()

	

