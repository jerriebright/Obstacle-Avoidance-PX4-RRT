#!/usr/bin/env python
from __future__ import print_function

import rospy
import rospy

from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandTOL

# arm or disarm
# state - 1 (arm) & state - 0 (disarm)
def arm(state):
    try:
        arming_cl = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        arming_cl(state)
    except rospy.ServiceException as ex:
        fault()

def takeoff():
    try:
        takeoff_cl = rospy.ServiceProxy("mavros/cmd/takeoff", CommandTOL)
        takeoff_cl(altitude=5, latitude=0, longitude=0, min_pitch=0, yaw=0)
    except rospy.ServiceException as ex:
        fault()

def land():
    try:
        land_cl = rospy.ServiceProxy("mavros/cmd/land", CommandTOL)
        land_cl(altitude=0, latitude=0, longitude=0, min_pitch=0, yaw=0)
    except rospy.ServiceException as ex:
        fault()

def fault():
	rospy.loginfo("exception detected")

def offboard():
    try:
        mavros_mode = rospy.ServiceProxy('mavros/set_mode', SetMode())
        mavros_mode(custom_mode='OFFBOARD')
    except rospy.ServiceException as ex:
        fault()