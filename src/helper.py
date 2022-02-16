import rospy
import numpy as np
import math
from geometry_msgs.msg import PoseStamped, Quaternion
from pymavlink import mavutil
from six.moves import xrange
from std_msgs.msg import Header
from threading import Thread
from tf.transformations import quaternion_from_euler

def set_arm(self, arm, timeout):
    rospy.loginfo("setting FCU arm: {0}".format(arm))
    loop_freq = 1  # Hz
    rate = rospy.Rate(loop_freq)
    for i in xrange(timeout * loop_freq):
        if self.state.armed == arm:
            rospy.loginfo("set arm success | seconds: {0} of {1}".format(
                i / loop_freq, timeout))
            break
        else:
            try:
                res = self.set_arming_srv(arm)
                if not res.success:
                    rospy.logerr("failed to send arm command")
            except rospy.ServiceException as e:
                rospy.logerr(e)
        try:
            rate.sleep()
        except rospy.ROSException as e:
            self.fail(e)

def set_mode(self, mode, timeout):
    rospy.loginfo("setting FCU mode: {0}".format(mode))
    old_mode = self.state.mode
    loop_freq = 1  # Hz
    rate = rospy.Rate(loop_freq)
    for i in xrange(timeout * loop_freq):
        if self.state.mode == mode:
            rospy.loginfo("set mode success | seconds: {0} of {1}".format(
                i / loop_freq, timeout))
            break
        else:
            try:
                res = self.set_mode_srv(0, mode)  # 0 is custom mode
                if not res.mode_sent:
                    rospy.logerr("failed to send mode command")
            except rospy.ServiceException as e:
                rospy.logerr(e)
        try:
            rate.sleep()
        except rospy.ROSException as e:
            self.fail(e)

def wait_for_topics(self, timeout):
    rospy.loginfo("waiting for subscribed topics to be ready")
    loop_freq = 1  # Hz
    rate = rospy.Rate(loop_freq)
    for i in xrange(timeout * loop_freq):
        if all(value for value in self.sub_topics_ready.values()):
            rospy.loginfo("simulation topics ready | seconds: {0} of {1}".
                            format(i / loop_freq, timeout))
            break
        try:
            rate.sleep()
        except rospy.ROSException as e:
            self.fail(e)

def wait_for_landed_state(self, desired_landed_state, timeout, index):
    rospy.loginfo("waiting for landed state | state: {0}, index: {1}".
                    format(mavutil.mavlink.enums['MAV_LANDED_STATE'][desired_landed_state].name, index))
    loop_freq = 10  # Hz
    rate = rospy.Rate(loop_freq)
    for i in xrange(timeout * loop_freq):
        if self.extended_state.landed_state == desired_landed_state:
            rospy.loginfo("landed state confirmed | seconds: {0} of {1}".format(i / loop_freq, timeout))
            break
        try:
            rate.sleep()
        except rospy.ROSException as e:
            self.fail(e)

def log_topic_vars(self):
    rospy.loginfo("========================")
    rospy.loginfo("===== topic values =====")
    rospy.loginfo("========================")
    rospy.loginfo("altitude:\n{}".format(self.altitude))
    rospy.loginfo("========================")
    rospy.loginfo("extended_state:\n{}".format(self.extended_state))
    rospy.loginfo("========================")
    rospy.loginfo("global_position:\n{}".format(self.global_position))
    rospy.loginfo("========================")
    rospy.loginfo("home_position:\n{}".format(self.home_position))
    rospy.loginfo("========================")
    rospy.loginfo("local_position:\n{}".format(self.local_position))
    rospy.loginfo("========================")
    rospy.loginfo("mission_wp:\n{}".format(self.mission_wp))
    rospy.loginfo("========================")
    rospy.loginfo("state:\n{}".format(self.state))
    rospy.loginfo("========================")

def setUp2(self,acc):

    self.pos = PoseStamped()
    self.radius = acc

    self.pos_setpoint_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=1)

    # send setpoints in seperate thread to better prevent failsafe
    self.pos_thread = Thread(target=self.send_pos, args=())
    self.pos_thread.daemon = True
    self.pos_thread.start()

def send_pos(self):
    rate = rospy.Rate(10)  # Hz
    self.pos.header = Header()
    self.pos.header.frame_id = "base_footprint"
    
    while not rospy.is_shutdown():
        self.pos.header.stamp = rospy.Time.now()
        self.pos_setpoint_pub.publish(self.pos)
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            pass

def is_at_position(self, x, y, z, offset):
    rospy.logdebug("current position | x:{0:.2f}, y:{1:.2f}, z:{2:.2f}".format(
            self.local_position.pose.position.x, self.local_position.pose. position.y, self.local_position.pose.position.z))
    desired = np.array((x, y, z))
    pos = np.array((self.local_position.pose.position.x, self.local_position.pose.position.y, self.local_position.pose.position.z))
    return np.linalg.norm(desired - pos) < offset

def reach_position(self, x, y, z, timeout):
    self.pos.pose.position.x = x
    self.pos.pose.position.y = y
    self.pos.pose.position.z = z
    rospy.loginfo("attempting to reach position | x: {0}, y: {1}, z: {2} | current position x: {3:.2f}, y: {4:.2f}, z: {5:.2f}".
        format(x, y, z, self.local_position.pose.position.x, self.local_position.pose.position.y, self.local_position.pose.position.z))
    yaw_degrees = 0  # lock heading to north
    yaw = math.radians(yaw_degrees)
    quaternion = quaternion_from_euler(0, 0, yaw)
    self.pos.pose.orientation = Quaternion(*quaternion)

    loop_freq = 2  # Hz
    rate = rospy.Rate(loop_freq)
    for i in xrange(timeout * loop_freq):
        if self.is_at_position(self.pos.pose.position.x, self.pos.pose.position.y, self.pos.pose.position.z, self.radius):
            rospy.loginfo("position reached | seconds: {0} of {1}".format(i / loop_freq, timeout))
            break
        try:
            rate.sleep()
        except rospy.ROSException as e:
            self.fail(e)