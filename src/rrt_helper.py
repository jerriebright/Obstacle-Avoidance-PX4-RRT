import numpy as np
import math, random 
from scipy import spatial
from sklearn.neighbors import KDTree
from mpl_toolkits.mplot3d import axes3d 

c = 1
length = 20
breadth = 15
height = 10

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
def obstacle_map(ax):
    # defining x, y, z co-ordinates for bar position 
    x = [5,8,12,10,16,3,17] 
    y = [5,11,14,2,7,10,3] 
    z = np.zeros(7)
    # size of bars 
    dx = np.ones(7)              # length along x-axis 
    dy = np.ones(7)              # length along y-axs 
    dz = [10,10,10,10,10,10,10]   # height of bar 
    # setting color scheme 
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
    
# Returns the point after maximum step propagation distance of 2m
def max_step_prop(j, i):
    k = (i[0] - j[0], i[1] - j[1], i[2] - j[2])
    k_mod = math.sqrt(k[0]**2 + k[1]**2 + k[2]**2)
    vec = (k[0]/k_mod, k[1]/k_mod, k[2]/k_mod)
    new_point = (j[0]+4*vec[0], j[1]+ 4*vec[1], j[2]+4*vec[2])
    return new_point

# Returns the set if neighbours for a seed in a given radius
def neighbours(seed, r, tree):
    results = tree.query_ball_point((seed), r)
    return results