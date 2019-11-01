#!/usr/bin/env python

class Code(object):

    def __init__(self):
        super(Code, self).__init__()

        # Do what is necessary to start the process
        # and to leave gloriously
        self.arm()

        self.sub = subs.Subs()
        self.pub = pubs.Pubs()

        self.sub.subscribe_topic('/joy', Joy)
        self.sub.subscribe_topic('/mavros/battery', BatteryState)
        self.sub.subscribe_topic('/mavros/rc/in', RCIn)
        self.sub.subscribe_topic('/mavros/rc/out', RCOut)

    def arm(self):
        """ Arm the vehicle and trigger the disarm
        """
        rospy.wait_for_service('/mavros/cmd/arming')

        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.arm_service(True)

        # Disarm is necessary when shutting down
        ##rospy.on_shutdown(self.disarm)lf.disarm)

    if __name__ == "__main__":
        try:
            rospy.init_node('user_node', log_level=rospy.DEBUG)
        except rospy.ROSInterruptException as error:
            print('pubs error with ROS: ', error)
            exit(1)
        code = Code()
        code.run()
