#
import mrob
import numpy as np


# Initialize data structures
vertex_ini = {}
factors = {}
factors_dictionary = {}
N = 30 #3500

# load file
with open('../../datasets/M3500.txt', 'r') as file:
    for line in file:
        d = line.split()
        # read edges and vertex
        if d[0] == 'EDGE2':
            # EDGE2 id_origin   id_target   dx   dy   dth   I11   I12  I22  I33  I13  I23
            factors[int(d[1])+1,int(d[2])+1] = np.array([d[3], d[4], d[5], d[6],d[7],d[8],d[9],d[10],d[11]],dtype='float64')
            factors_dictionary[int(d[2])+1].append(int(d[1])+1)
        else:
            # VERTEX2 id x y theta
            # these are the initial guesses for node states
            vertex_ini[int(d[1])+1] = np.array([d[2], d[3], d[4]],dtype='float64')
            # create an empty list of pairs of nodes (factor) connected to each node
            factors_dictionary[int(d[1]) + 1] = []


        # Initialize FG
graph = mrob.FGraph(3500,5500)
x = np.zeros(3)
n = graph.add_node_pose_2d(x)
print('node 0 id = ', n) # id starts at 1
graph.add_factor_1pose_2d(x,n,1e9*np.identity(3))


# start events, we solve for each node, adding it and it corresponding factors
for t in range(2,N+1):
    x = vertex_ini[t]
    n = graph.add_node_pose_2d(x) #this value will be updated later with the odometry factor
    assert t == n, 'index on node is different from counter'

    # find factors to add. there must be 1 odom and other observations
    connecting_nodes = factors_dictionary[n]

    for nodeOrigin in factors_dictionary[n]:
        #if nodeOrigin == n - 1: # this is an odometry factor
        # inputs: obs, idOrigin, idTarget, invCov
        obs = factors[nodeOrigin, t][:3]
        covInv = np.zeros((3,3))
        # on M3500 always diagonal information matrices
        covInv[0,0] = factors[nodeOrigin, t][3]
        covInv[1,1] = factors[nodeOrigin, t][5]
        covInv[2,2] = factors[nodeOrigin, t][6]
        graph.add_factor_2poses_2d(obs, nodeOrigin,t,covInv)

        # solve the problem
        graph.solve_batch()

        # print the problem



graph.print(True)

