#!/usr/bin/env python2

from __future__ import division

import rospy, math, random, copy
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped
from pymavlink import mavutil
from six.moves import xrange
from mavros_msgs.msg import Altitude, ExtendedState, HomePosition, State, WaypointList
from mavros_msgs.srv import CommandBool, SetMode
from sensor_msgs.msg import NavSatFix
from sklearn.neighbors import KDTree
from mpl_toolkits.mplot3d import axes3d 
from scipy import spatial
from queue import PriorityQueue 

import helper

q = PriorityQueue() 
plt.ion()
length = 20
breadth = 15
height = 10
c = 1

# To check if a point is in obstacle space or not
def obstacle_check(i,j,k):
    a = b = c = d = e = f = g = 0
    if(5-c <= i <=6+c) and (5-c <= j <= 6+c) and (0 <= k <= 10):
        a = 1
    elif (8-c <= i <=9+c) and (11-c <= j <= 12+c) and (0 <= k <= 10):
        b = 1
    elif (12-c <= i <=13+c) and (14-c <= j <= 15+c) and (0 <= k <= 10):
        c = 1
    elif (10-c <= i <11+c) and (2-c <= j <= 3+c) and (0 <= k <= 10):
        d = 1
    elif (16-c <= i <=17+c) and (7-c <= j <= 8+c) and (0 <= k <= 10):
        e = 1
    elif (3-c <= i <=4+c) and (10-c <= j <= 11+c) and (0 <= k <= 10):
        f = 1
    elif (17-c <= i <=18+c) and (3-c <= j <= 4+c) and (0 <= k <= 10):
        g = 1

    if  ((a == 1) or (b == 1) or (c == 1) or (d == 1) or (e == 1) or (f == 1) or (g == 1) ):
        return True
    else:
        return False

# Plots the obstacles in the 3D map
def obstacle_map():
    x = [5,8,12,10,16,3,17]      # x-coord of bar
    y = [5,11,14,2,7,10,3]       # y-coord of bar
    z = np.zeros(7)
    dx = np.ones(7)              # length of bar
    dy = np.ones(7)              # breadth of bar
    dz = [10,10,10,10,10,10,10]  # height of bar 
    color = [] 
    for h in dz: 
        if h > 5: 
            color.append('b') 
        else: 
            color.append('b')
    ax.bar3d(x, y, z, dx, dy, dz, color = color)

# Boundary check
def boundary_check(i, j, k):
    if (i < 0) or (j < 0) or (k < 0) or (i >= length) or (j >= breadth) or (k >= height):
        return True
    else:
        return False

# Returns the point 0.1m away from parent to child
def line_obstacle_check(j, i):
    k = (i[0] - j[0], i[1] - j[1], i[2] - j[2])
    k_mod = math.sqrt(k[0]**2 + k[1]**2 + k[2]**2)
    vec = (k[0]/k_mod, k[1]/k_mod, k[2]/k_mod)
    new_point = (j[0]+0.1*vec[0], j[1]+ 0.1*vec[1], j[2]+0.1*vec[2])
    return new_point

# Returns Euclidean distance between two 3D points
def cost2go(pt1, pt2):
    dist = math.sqrt((pt2[0] - pt1[0]) ** 2 + (pt2[1] - pt1[1]) ** 2 + (pt2[2] - pt1[2]) ** 2) 
    return dist

# Generates a random seed in 3D rounded off to 0.5
def generate_seed():
    x = round(random.uniform(0 , length)*2)/2
    y = round(random.uniform(0 , breadth)*2)/2
    z = round(random.uniform(0 , height)*2)/2
    return (x,y,z)
    
# Sphere to check Goal convergence 
def goalcheck_circle(x, y, z, goal_x, goal_y, goal_z):
    if ((x - goal_x) ** 2 + (y - goal_y) ** 2 + (z - goal_z) **2 <= (0.5 ** 2)):
        return True
    else:
        return False
    
# Returns the point after maximum step propagation disance of 2m
def max_step_prop(j, i):
    k = (i[0] - j[0], i[1] - j[1], i[2] - j[2])
    k_mod = math.sqrt(k[0]**2 + k[1]**2 + k[2]**2)
    vec = (k[0]/k_mod, k[1]/k_mod, k[2]/k_mod)
    new_point = (j[0]+4*vec[0], j[1]+ 4*vec[1], j[2]+4*vec[2])
    return new_point

# Returns the set if neighbours for a seed in a given radius
def neighbours(seed, r, tree):
    results = tree.query_ball_point((seed), r)
    nearby_points = X[results]
    return nearby_points

# Test algo
def test_posctl(self,positions):
    self.wait_for_topics(60)
    self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 10, -1)
    self.log_topic_vars()
    self.set_mode("OFFBOARD", 5)
    self.set_arm(True, 5)
    rospy.loginfo("run mission")
    for i in xrange(len(positions)):
        self.reach_position(positions[i][0], positions[i][1], positions[i][2], 10)
    self.set_mode("AUTO.LAND", 2)
    self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 15, 0)
    self.set_arm(False, 2)

def tearDown(self):
    super(missionfly, self).tearDown()

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

    def altitude_callback(self, data):
        self.altitude = data
        if not self.sub_topics_ready['alt'] and not math.isnan(data.amsl):
            self.sub_topics_ready['alt'] = True

    def extended_state_callback(self, data):
        if self.extended_state.vtol_state != data.vtol_state:
            rospy.loginfo("VTOL state changed from {0} to {1}".format(mavutil.mavlink.enums['MAV_VTOL_STATE']
                [self.extended_state.vtol_state].name, mavutil.mavlink.enums['MAV_VTOL_STATE'][data.vtol_state].name))

        if self.extended_state.landed_state != data.landed_state:
            rospy.loginfo("landed state changed from {0} to {1}".format(mavutil.mavlink.enums['MAV_LANDED_STATE']
                [self.extended_state.landed_state].name, mavutil.mavlink.enums['MAV_LANDED_STATE'][data.landed_state].name))
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
            rospy.loginfo("current mission waypoint sequence updated: {0}".format(data.current_seq))
        self.mission_wp = data
        if not self.sub_topics_ready['mission_wp']:
            self.sub_topics_ready['mission_wp'] = True

    def state_callback(self, data):
        if self.state.armed != data.armed:
            rospy.loginfo("armed state changed from {0} to {1}".format(self.state.armed, data.armed))
        if self.state.connected != data.connected:
            rospy.loginfo("connected changed from {0} to {1}".format(self.state.connected, data.connected))
        if self.state.mode != data.mode:
            rospy.loginfo("mode changed from {0} to {1}".format(self.state.mode, data.mode))
        if self.state.system_status != data.system_status:
            rospy.loginfo("system_status changed from {0} to {1}".format(mavutil.mavlink.enums['MAV_STATE'][
                    self.state.system_status].name, mavutil.mavlink.enums['MAV_STATE'][data.system_status].name))
        self.state = data
        if not self.sub_topics_ready['state'] and data.connected:
            self.sub_topics_ready['state'] = True
    
start = (0,0,0)
goal_x, goal_y, goal_z = (20,10,6)
visited_nodes = set()
all_nodes = []
parent_list = []
seed_list = []
parent_dict = {}
cost_dict = {}
quadpos=[]
path = []

visited_nodes.add(start)
all_nodes.append(start)
parent_dict[start] = "okay"
cost_dict[start] = 0
accuracy = 0.3
seed = (0,0,0)
i = l = 0

while(goalcheck_circle(goal_x, goal_y, goal_z, seed[0], seed[1], seed[2]) == False):
    seed = generate_seed()

    if ((seed not in visited_nodes) and not obstacle_check(seed[0], seed[1], seed[2])):
        
        X = np.asarray(all_nodes)
        tree = spatial.KDTree(X)
        
        r = 2
        n = (0,0,0)

        while(1):
            n = neighbours(seed, r,tree)
            if(n == seed).all():
                r = r + 1
            else:
                break        
        
        for pt in n:
            pt = tuple(pt)
            cost = cost_dict[pt]
            cost_new = cost + cost2go(pt, seed)
            q.put((cost_new, pt, cost))
        
        parent = q.get()
        q = PriorityQueue() 
        parent = parent[1] 
                    
        if (cost2go(parent,seed) > 4):
            seed = max_step_prop(parent, seed)
            seed = (round(seed[0], 1), round(seed[1], 1), round(seed[2], 1))            
            
        par = seed
        s = parent
        a = 0

        while(cost2go(par,s)>=0.1):
            a = line_obstacle_check(s, par)
            if obstacle_check(a[0], a[1], a[2]):
                break
            s = a

        s = (round(s[0], 1), round(s[1], 1), round(s[2], 1))
        
        if s not in visited_nodes:
            neww_cost = cost2go(seed, parent) + cost_dict[parent]  
            all_nodes.insert(0, s)
            visited_nodes.add(s)
            parent_dict[s] = parent 
            cost_dict[s] = neww_cost
            parent_list.append((parent[0], parent[1], parent[2]))
            seed_list.append((s[0], s[1], s[2]))

            for nei in n:
                nei = tuple(nei)
                if nei != parent:
                    if cost_dict[nei] > (cost_dict[s] + cost2go(s, nei)):
                        parent_dict[nei] = s
                        cost_dict[nei] = cost_dict[s] + cost2go(s, nei)
        else:
            all_nodes.pop(0)

path.append((s[0], s[1], s[2]))
while parent != 'okay':
    temp = parent_dict.get(parent)
    path.append(parent)
    parent = temp
    if parent == (start):
        break
path.append(start)

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
        
        while(cost2go(choice2, choice1)>=0.1):
            a = line_obstacle_check(choice1, choice2)
            if obstacle_check(a[0], a[1], a[2]):
                break
            choice1 = a
        choice2 = ch1
    if (cost2go(choice1, ch2)) < 0.2:
        if ind1< ind2:
            del path[ind1+1:ind2]

        if ind2 < ind1:
            del path[ind2+1:ind1]

    print("after_optimise")
    print("choice1", choice2)
    print("choice2", choice1)

    print("\n")

fig = plt.figure()
ax = plt.axes(projection='3d')
ax.set_xlim3d(0, 20)
ax.set_ylim3d(0,15)
ax.set_zlim3d(0,10)
obstacle_map()
    
x_path_original = [ori_path[i][0] for i in range(len(ori_path))]
y_path_original = [ori_path[i][1] for i in range(len(ori_path))]
z_path_original = [ori_path[i][2] for i in range(len(ori_path))]
x_path = [path[i][0] for i in range(len(path))]
y_path = [path[i][1] for i in range(len(path))]
z_path = [path[i][2] for i in range(len(path))]

ax.scatter(x_path, y_path, z_path, c='g', marker = 'o')
ax.plot3D(x_path, y_path, z_path, "-r")

ax.plot3D(x_path_original, y_path_original, z_path_original, "-g") # Path before optimisation

while l < len(seed_list): # Real time exploration of the tree
    ax.plot3D((parent_list[l][0], seed_list[l][0]), (parent_list[l][1], seed_list[l][1]), (parent_list[l][2], seed_list[l][2]), 'black')
    l = l + 1
    plt.show()
    plt.pause(0.000000000000000000000000000000000001)

print("before", ori_path)
print("after", path)

rospy.init_node('obstacle_avoidance_rrt', anonymous=True)

for i in path:
    quadpos.append((-i[1],i[0],i[2]))

drone1=missionfly()
drone1.setUp()
drone1.setUp2(accuracy)
drone1.set_arm(True,10)
drone1.set_mode("OFFBOARD",50)
drone1.test_posctl(quadpos)