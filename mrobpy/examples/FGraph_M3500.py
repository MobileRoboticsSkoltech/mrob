#
import mrob
import numpy as np


# Initialize data structures
vertex_gt = {}
edges = {}
N = 3500

# load file
with open('../../datasets/M3500.txt', 'r') as file:
    for line in file:
        d = line.split()
        # read edges and vertex
        if d[0] == 'EDGE2':
            # EDGE2 id1 id2 x y theta covs
            edges[int(d[1])+1,int(d[2])+1] = np.array([d[3], d[4], d[5], d[6],d[7],d[8],d[9],d[10],d[11]],dtype='float64')
        else:
            # VERTEX2 id x y theta
            vertex_gt[int(d[1])+1] = np.array([d[2], d[3], d[4]],dtype='float64')


# Initialize FG
graph = mrob.FGraph()
x = np.zeros(3)
n = graph.add_node_pose_2d(x)
print('node 0 id = ', n) # id starts at 1
graph.add_factor_1pose_2d(x,n,1e9*np.identity(3))


# start events, we solve for each node, adding it and it corresponding factors
for t in range(2,N+1):
    graph.add_node_pose_2d(x) #this value will be updated later with the odometry factor

    # find factors to add. there must be 1 odom and other observations
    edges.keys()
    # inputs: obs, idOrigin, idTarget, invCov
    graph.add_factor_2poses_2d_odom()


graph.print(True)

