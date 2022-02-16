#!/usr/bin/env python2

from __future__ import division

import rospy, math, random, copy
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion
from pymavlink import mavutil
from six.moves import xrange
from mavros_msgs.msg import Altitude, ExtendedState, HomePosition, State, WaypointList
from mavros_msgs.srv import CommandBool, SetMode
from sensor_msgs.msg import NavSatFix
from sklearn.neighbors import KDTree
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d 
import matplotlib.pyplot as plt 
from scipy import spatial
from queue import PriorityQueue 
from threading import Thread
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler

import rrt_helper

q = PriorityQueue() 
plt.ion()

length = 20
breadth = 15
height = 10
c = 1

class missionfly():
    def setUp(self):
        self.altitude = Altitude()
        self.extended_state = ExtendedState()
        self.global_position = NavSatFix()
        self.home_position = HomePosition()
        self.local_position = PoseStamped()
        self.mission_wp = WaypointList()
        self.state = State()
        self.mav_type = None

        self.sub_topics_ready = {
            key: False
            for key in ['alt', 'ext_state', 'global_pos', 'home_pos', 'local_pos', 'mission_wp', 'state']
        }

        service_timeout = 30
        try:
            rospy.wait_for_service('mavros/param/get', service_timeout)
            rospy.wait_for_service('mavros/cmd/arming', service_timeout)
            rospy.wait_for_service('mavros/mission/push', service_timeout)
            rospy.wait_for_service('mavros/mission/clear', service_timeout)
            rospy.wait_for_service('mavros/set_mode', service_timeout)
            rospy.loginfo("ROS services are up")
        except rospy.ROSException:
            self.fail("failed to connect to services")

        self.set_arming_srv = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        self.set_mode_srv = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.alt_sub = rospy.Subscriber('mavros/altitude', Altitude, self.altitude_callback)
        self.ext_state_sub = rospy.Subscriber('mavros/extended_state', ExtendedState, self.extended_state_callback)
        self.global_pos_sub = rospy.Subscriber('mavros/global_position/global', NavSatFix, self.global_position_callback)
        self.home_pos_sub = rospy.Subscriber('mavros/home_position/home', HomePosition, self.home_position_callback)
        self.local_pos_sub = rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.local_position_callback)
        self.mission_wp_sub = rospy.Subscriber( 'mavros/mission/waypoints', WaypointList, self.mission_wp_callback)
        self.state_sub = rospy.Subscriber('mavros/state', State, self.state_callback)

    def tearDown(self):
        self.log_topic_vars()

    ################################################### CALLBACK FUNCTIONS ###################################################
    def altitude_callback(self, data):
        self.altitude = data
        if not self.sub_topics_ready['alt'] and not math.isnan(data.amsl):
            self.sub_topics_ready['alt'] = True

    def extended_state_callback(self, data):
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
            rospy.loginfo("Mission waypoints updated: {0}".format(data.current_seq))
        self.mission_wp = data
        if not self.sub_topics_ready['mission_wp']:
            self.sub_topics_ready['mission_wp'] = True

    def state_callback(self, data):
        if self.state.armed != data.armed:
            rospy.loginfo("Arming changed from {0} to {1}".format(self.state.armed, data.armed))
        if self.state.connected != data.connected:
            rospy.loginfo("Connection changed from {0} to {1}".format(self.state.connected, data.connected))
        if self.state.mode != data.mode:
            rospy.loginfo("Mode changed from {0} to {1}".format(self.state.mode, data.mode))
        if self.state.system_status != data.system_status:
            rospy.loginfo("System status changed from {0} to {1}".format(mavutil.mavlink.enums['MAV_STATE'][
                    self.state.system_status].name, mavutil.mavlink.enums['MAV_STATE'][data.system_status].name))
        self.state = data
        if not self.sub_topics_ready['state'] and data.connected:
            self.sub_topics_ready['state'] = True

    ################################################### HELPER FUNCTIONS ###################################################
    def set_arm(self, arm, timeout):
        rospy.loginfo("ARM: {0}".format(arm))
        loop_freq = 1
        rate = rospy.Rate(loop_freq)
        for i in xrange(timeout * loop_freq):
            if self.state.armed == arm:
                rospy.loginfo("ARMED ({0}/{1})".format(i / loop_freq, timeout))
                break
            else:
                try:
                    res = self.set_arming_srv(arm)
                    if not res.success:
                        rospy.logerr("Failed to send arm command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)
            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

    def set_mode(self, mode, timeout):
        rospy.loginfo("MODE: {0}".format(mode))
        loop_freq = 1
        rate = rospy.Rate(loop_freq)
        for i in xrange(timeout * loop_freq):
            if self.state.mode == mode:
                rospy.loginfo("MODE SET".format(mode))
                break
            else:
                try:
                    res = self.set_mode_srv(0, mode)
                    if not res.mode_sent:
                        rospy.logerr("Failed to send mode command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)
            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

    def send_pos(self):
        rate = rospy.Rate(10)
        self.pos.header = Header()
        self.pos.header.frame_id = "base_footprint"        
        while not rospy.is_shutdown():
            self.pos.header.stamp = rospy.Time.now()
            self.pos_setpoint_pub.publish(self.pos)
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                pass
    
    def setUp2(self,acc):
        self.pos = PoseStamped()
        self.radius = acc
        self.pos_setpoint_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=1)
        self.pos_thread = Thread(target=self.send_pos, args=())
        self.pos_thread.daemon = True
        self.pos_thread.start()

    def is_at_position(self, x, y, z, offset):
        desired = np.array((x, y, z))
        pos = np.array((self.local_position.pose.position.x, self.local_position.pose.position.y, self.local_position.pose.position.z))
        return np.linalg.norm(desired - pos) < offset

    def reach_position(self, x, y, z, timeout):
        self.pos.pose.position.x = x
        self.pos.pose.position.y = y
        self.pos.pose.position.z = z
        rospy.loginfo("x: {3:.2f}, y: {4:.2f}, z: {5:.2f} to x: {0}, y: {1}, z: {2}".
            format(x, y, z, self.local_position.pose.position.x, self.local_position.pose.position.y, self.local_position.pose.position.z))
        yaw_degrees = 0  # lock heading to north
        yaw = math.radians(yaw_degrees)
        quaternion = quaternion_from_euler(0, 0, yaw)
        self.pos.pose.orientation = Quaternion(*quaternion)

        loop_freq = 2
        rate = rospy.Rate(loop_freq)
        for i in xrange(timeout * loop_freq):
            if self.is_at_position(self.pos.pose.position.x, self.pos.pose.position.y, self.pos.pose.position.z, self.radius):
                rospy.loginfo("Reached: {0} of {1}".format(i / loop_freq, timeout))
                break
            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

    ################################################### IMPLEMENTATION ###################################################
    def test_posctl(self,positions):
        rospy.loginfo("==================================== OFFBOARD MODE INITIATE ====================================")
        self.set_mode("OFFBOARD", 5)
        self.set_arm(True, 5)
        rospy.loginfo("======================================== START MISSION =========================================")
        for i in xrange(len(positions)):
            if self.state.mode != False:
                self.set_arm(True, 10)
            self.reach_position(positions[i][0], positions[i][1], positions[i][2], 10)
        rospy.loginfo("=================================== LAND AND DISARM INITIATE ===================================")
        self.set_mode("AUTO.LAND", 2)
        self.set_arm(False, 2)
        rospy.loginfo("======================================= MISSION COMPLETE =======================================")

################################################### RRT PATH PLANNING ###################################################
start = (0,0,0)
goal_x, goal_y, goal_z = (20,10,6)
visited_nodes = set()
all_nodes = []
parent_list = []
seed_list = []
parent_dict = {}
cost_dict = {}
quadpos = []

visited_nodes.add(start)
all_nodes.append(start)
parent_dict[start] = "okay"
cost_dict[start] = 0
i = 0
l = 0
seed = (0,0,0)
accuracy = 0.3

while(rrt_helper.goalcheck_circle(goal_x, goal_y, goal_z, seed[0], seed[1], seed[2]) == False):
    seed = rrt_helper.generate_seed()

    if ((seed not in visited_nodes) and not rrt_helper.obstacle_check(seed[0], seed[1], seed[2])):
        
        X = np.asarray(all_nodes)
        tree = spatial.KDTree(X)        
        r = 2
        n = (0,0,0)

        while(1):
            ngh = rrt_helper.neighbours(seed, r, tree)
            n = X[ngh]
            if(n == seed).all():
                r = r + 1
            else:
                break        
        
        for pt in n:
            pt = tuple(pt)
            cost = cost_dict[pt]
            cost_new = cost + rrt_helper.cost2go(pt, seed)
            q.put((cost_new, pt, cost))
        
        parent = q.get()
        q = PriorityQueue() 
        parent = parent[1] 
                     
        if (rrt_helper.cost2go(parent,seed) > 4):
            seed = rrt_helper.max_step_prop(parent, seed)
            seed = (round(seed[0], 1), round(seed[1], 1), round(seed[2], 1))            
            
        par = seed
        s = parent
        a = 0

        while(rrt_helper.cost2go(par,s)>=0.1):
            a = rrt_helper.line_obstacle_check(s, par)
            if rrt_helper.obstacle_check(a[0], a[1], a[2]):
                break
            s = a

        s = (round(s[0], 1), round(s[1], 1), round(s[2], 1))
         
        if s not in visited_nodes:
            neww_cost = rrt_helper.cost2go(seed, parent) + cost_dict[parent]  
            all_nodes.insert(0, s)
            visited_nodes.add(s)
            parent_dict[s] = parent 
            cost_dict[s] = neww_cost
            parent_list.append((parent[0], parent[1], parent[2]))
            seed_list.append((s[0], s[1], s[2]))

            for nei in n:
                nei = tuple(nei)
                if nei != parent:
                    if cost_dict[nei] > (cost_dict[s] + rrt_helper.cost2go(s, nei)):
                        parent_dict[nei] = s
                        cost_dict[nei] = cost_dict[s] + rrt_helper.cost2go(s, nei)
        else:
            all_nodes.pop(0)

path = []
path.append((s[0], s[1], s[2]))
while parent != 'okay':
    temp = parent_dict.get(parent)
    path.append(parent)
    parent = temp
    if parent == (start):
        break
path.append(start)
print("Shortest path found")

path = path[::-1]
ori_path = copy.deepcopy(path)

for i in range(1):
    choice1 = random.choice(path)
    choice2 = random.choice(path)
    ch1 = choice1
    ch2 = choice2                    
    ind1 = path.index(choice1)
    ind2 = path.index(choice2)

    if (choice1 != choice2):        
        while(rrt_helper.cost2go(choice2, choice1)>=0.1):
            a = rrt_helper.line_obstacle_check(choice1, choice2)
            if rrt_helper.obstacle_check(a[0], a[1], a[2]):
                break
            choice1 = a
        choice2 = ch1
    if (rrt_helper.cost2go(choice1, ch2)) < 0.2:
        if ind1< ind2:
            del path[ind1+1:ind2]
        if ind2 < ind1:
            del path[ind2+1:ind1]
    print("\n")

################################################### VISUALIZATION ###################################################
fig = plt.figure()
ax = plt.axes(projection='3d')
ax.set_xlim3d(0, 20)
ax.set_ylim3d(0,15)
ax.set_zlim3d(0,10)
rrt_helper.obstacle_map(ax)
    
x_path_ori = [ori_path[i][0] for i in range(len(ori_path))]
y_path_ori = [ori_path[i][1] for i in range(len(ori_path))]
z_path_ori = [ori_path[i][2] for i in range(len(ori_path))]
x_path = [path[i][0] for i in range(len(path))]
y_path = [path[i][1] for i in range(len(path))]
z_path = [path[i][2] for i in range(len(path))]

ax.scatter(x_path, y_path, z_path, c='g', marker = 'o')
ax.plot3D(x_path, y_path, z_path, "-r")
ax.plot3D(x_path_ori, y_path_ori, z_path_ori, "-g") # Path before optimisation

while l < len(seed_list): # Real time exploration of the tree
    ax.plot3D((parent_list[l][0], seed_list[l][0]), (parent_list[l][1], seed_list[l][1]), (parent_list[l][2], seed_list[l][2]),  'black')
    l = l + 1
    plt.show()
    plt.pause(0.000000000000000000000000000000000001)

print("Before", ori_path)
print("After", path)
ax.set_xlabel('x-axis') 
ax.set_ylabel('y-axis') 
ax.set_zlabel('z-axis') 

rospy.init_node('obstacle_avoidance_rrt', anonymous=True)

for i in path:
    quadpos.append((-i[1],i[0],i[2]))

################################################### IMPLEMENTATION ###################################################
drone1 = missionfly()
drone1.setUp()
drone1.setUp2(accuracy)
drone1.set_arm(True, 10)
drone1.set_mode("OFFBOARD", 50)
drone1.test_posctl(quadpos)