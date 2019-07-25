#!/usr/bin/env python

import cv2
import rospy
import time

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
from sensor_msgs.msg import JointState, Joy

from sensor_msgs.msg import BatteryState, FluidPressure
from mavros_msgs.msg import OverrideRCIn, RCIn, RCOut


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

        self.pub.subscribe_topic('/mavros/rc/override', OverrideRCIn)
        self.pub.subscribe_topic('/mavros/setpoint_velocity/cmd_vel', TwistStamped)
        self.pub.subscribe_topic('/BlueRov2/body_command', JointState)

        self.sub.subscribe_topic('/joy', Joy)
        self.sub.subscribe_topic('/mavros/battery', BatteryState)
        self.sub.subscribe_topic('/mavros/rc/in', RCIn)
        self.sub.subscribe_topic('/mavros/rc/out', RCOut)

        self.sub.subscribe_topic('/mavros/imu/static_pressure', FluidPressure)
        self.sub.subscribe_topic('/mavros/imu/diff_pressure', FluidPressure)

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

    def apply_control(self, forces, torques, camupdown):
        """Sends control input to the robot by setting appropriate levels of PWM
           in the /mavros/rc/override
           Inputs: -1.0...1.0 for each of the force and torque axis
                    (it corresponds to the desired velocity level)
                    (the unit is arbitrary)
                    camupdown is in the same range, deciding if the camera should be
                    pitched a step down or a step up
        """
        # rc run between 1100 and 2000, a joy command is between -1.0 and 1.0
        # Correction SK:  1100 and 1900
        #override = [int(val*400 + 1500) for val in joy]
        #for _ in range(len(override), 8):
        #    override.append(0)
        override = [1500,1500,1500,1500,1500,1500,1500,1500]
        # Not implemented in MAVLink apparently:
        #override = [1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500]
        # Surge = Channel 5
        override[4] = int(forces[0]*400 + 1500)
        # Sway = Channel 6
        override[5] = int(forces[1]*400 + 1500)
        # Heave = Channel 3
        override[2] = int(forces[2]*400 + 1500)
        # Roll = Channel 2
        override[1] = int(torques[0]*400 + 1500)
        # Pitch = Channel 1
        override[0] = int(torques[1]*400 + 1500)
        # Yaw = Channel 4
        override[3] = int(torques[2]*400 + 1500)
        # Cam up = Channel 8
        override[7] = int(camupdown*400 + 1500)

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
        

    def run(self):
        """Run user code
        """
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
                
                camsetpt = 0.0

		# Converting button presses to roll and pitch setting modification
		self.curr_pitch_setting = self.enforce_limit(self.curr_pitch_setting - joy[7]*0.1)
		self.curr_roll_setting = self.enforce_limit(self.curr_roll_setting - joy[6]*0.1)
		if (but[4] == 1):
		    self.curr_pitch_setting = 0.0
		if (but[5] == 1):
		    self.curr_roll_setting = 0.0                    
                # Camera

                camsetpt = but[4]-but[5]


		forces = [0.0, 0.0, 0.0]
		torques = [0.0, 0.0, 0.0]

		forces[0] = joy[4] #Surge
		forces[1] = -joy[3] #Sway
		forces[2] = joy[1] #Heave
		torques[0] = self.curr_roll_setting #Roll
		torques[1] = self.curr_pitch_setting #Pitch
		torques[2] = -joy[0] #Yaw

		self.apply_control(forces, torques, camsetpt)

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
        rospy.init_node('user_node', log_level=rospy.DEBUG)
    except rospy.ROSInterruptException as error:
        print('pubs error with ROS: ', error)
        exit(1)
    code = Code()
    code.run()
