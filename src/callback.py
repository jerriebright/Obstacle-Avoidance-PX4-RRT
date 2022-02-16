import math
import rospy
from pymavlink import mavutil

def altitude_callback(self, data):
    self.altitude = data

    # amsl has been observed to be nan while other fields are valid
    if not self.sub_topics_ready['alt'] and not math.isnan(data.amsl):
        self.sub_topics_ready['alt'] = True

def extended_state_callback(self, data):
    if self.extended_state.vtol_state != data.vtol_state:
        rospy.loginfo("VTOL state changed from {0} to {1}".format(
            mavutil.mavlink.enums['MAV_VTOL_STATE']
            [self.extended_state.vtol_state].name, mavutil.mavlink.enums[
                'MAV_VTOL_STATE'][data.vtol_state].name))

    if self.extended_state.landed_state != data.landed_state:
        rospy.loginfo("landed state changed from {0} to {1}".format(
            mavutil.mavlink.enums['MAV_LANDED_STATE']
            [self.extended_state.landed_state].name, mavutil.mavlink.enums[
                'MAV_LANDED_STATE'][data.landed_state].name))

    self.extended_state = data

    if not self.sub_topics_ready['ext_state']:
        self.sub_topics_ready['ext_state'] = True

def global_position_callback(self, data):
    self.global_position = data

    if not self.sub_topics_ready['global_pos']:
        self.sub_topics_ready['global_pos'] = True

def home_position_callback(self, data):
    self.home_position = data

    if not self.sub_topics_ready['home_pos']:
        self.sub_topics_ready['home_pos'] = True

def local_position_callback(self, data):
    self.local_position = data

    if not self.sub_topics_ready['local_pos']:
        self.sub_topics_ready['local_pos'] = True

def mission_wp_callback(self, data):
    if self.mission_wp.current_seq != data.current_seq:
        rospy.loginfo("current mission waypoint sequence updated: {0}".
                        format(data.current_seq))

    self.mission_wp = data

    if not self.sub_topics_ready['mission_wp']:
        self.sub_topics_ready['mission_wp'] = True

def state_callback(self, data):
    if self.state.armed != data.armed:
        rospy.loginfo("armed state changed from {0} to {1}".format(
            self.state.armed, data.armed))

    if self.state.connected != data.connected:
        rospy.loginfo("connected changed from {0} to {1}".format(
            self.state.connected, data.connected))

    if self.state.mode != data.mode:
        rospy.loginfo("mode changed from {0} to {1}".format(
            self.state.mode, data.mode))

    if self.state.system_status != data.system_status:
        rospy.loginfo("system_status changed from {0} to {1}".format(
            mavutil.mavlink.enums['MAV_STATE'][
                self.state.system_status].name, mavutil.mavlink.enums[
                    'MAV_STATE'][data.system_status].name))

    self.state = data

    # mavros publishes a disconnected state message on init
    if not self.sub_topics_ready['state'] and data.connected:
        self.sub_topics_ready['state'] = True