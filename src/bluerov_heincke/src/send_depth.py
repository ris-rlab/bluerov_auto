#!/usr/bin/env python

from __future__ import print_function
# ROS module
import rospy
# Mavlink ROS messages
from mavros_msgs.msg import Mavlink
# pack and unpack functions to deal with the bytearray
from struct import pack, unpack
import requests
import argparse
import time
import logging

log = logging.getLogger()
logging.basicConfig(format='%(asctime)s %(message)s', level=logging.INFO)

# Topic callback
def callback(data):
    # Check if message id is valid
    if data.msgid == 137:
        rospy.loginfo(rospy.get_caller_id() + " Package: %s", data)
        # Transform the payload in a python string
        p = pack("QQ", *data.payload64)
        # Transform the string in valid values
        # https://docs.python.org/2/library/struct.html
        time_boot_ms, press_abs, press_diff, temperature = unpack("Iffhxx", p)
        # print(time_boot_ms, press_abs, press_diff, temperature)

        depth = (1013 - press_abs) / 98.0665
        print('depth:', depth, 'm')
        temp = temperature
        url = 'http://demo.waterlinked.com/api/v1/external/depth' #'{}/api/v1/external/depth'.format(baseurl)

        payload = dict(depth=depth, temp=temp)
        r = requests.put(url, json=payload, timeout=10)
        if r.status_code != 200:
            log.error("Error setting depth: {} {}".format(r.status_code, r.text))

#def set_depth(url, depth, temp):
#    payload = dict(depth=depth, temp=temp)
#    r = requests.put(url, json=payload, timeout=10)
#    if r.status_code != 200:
#        log.error("Error setting depth: {} {}".format(r.status_code, r.text))
    
def listener():

#    parser = argparse.ArgumentParser(description=__doc__)
#    parser.add_argument('-u', '--url', help='Base URL to use', type=str, default='http://demo.waterlinked.com')
#    args = parser.parse_args()
#    baseurl = args.url    

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/mavlink/from", Mavlink, callback)

#    set_depth('{}/api/v1/external/depth'.format(baseurl), depth, temp)

    rospy.spin()

if __name__ == '__main__':
    listener()
