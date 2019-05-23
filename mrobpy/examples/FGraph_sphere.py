#
import mrob
import numpy as np
import time

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D



def print_3d_graph(graph):
    '''This function draws the state variables for a 3D pose graph'''
    
    # read graph, returns a list (vector) of state (np arrays)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    x = graph.get_estimated_state()
    prev_p = np.array([18.7381, 2.74428e-07, 98.2287], dtype='float64') #first position accoring to dataset TODO read first
    for xi in x:
        Ti = mrob.SE3(xi)
        p = Ti.T()[:3,3]
        ax.plot((prev_p[0],p[0]),(prev_p[1],p[1]),(prev_p[2],p[2]) , '-b')
        prev_p = np.copy(p)
    plt.show()




# Initialize data structures
vertex_ini = {}
factors = {}
factor_inf = {}
factors_dictionary = {}
N = 2200

# load file, .g2o format from https://github.com/RainerKuemmerle/g2o/wiki/File-Format
with open('../../datasets/sphere_bignoise_vertex3.g2o', 'r') as file:
    for line in file:
        d = line.split()
        # read edges and vertex
        if d[0] == 'EDGE_SE3:QUAT':
            # EDGE_SE33:QUAT id_origin[1]  id_target[2]   x[3],y[4],z[5],qx[6],qy[7],qz[8],qw[9]
            #     L11[10], L12[11], L13, L14, L15, L16
            #              L22[16], L23, L24, L25, L26
            #                       L33[21], L34, L35, L36
            #                                L44[25], L45 L46
            #                                       L55[28] L56
            #                                             L66[30]

            # transforming quaterntion to SE3
            quat = np.array([d[6],d[7],d[8],d[9]],dtype='float64')
            T = np.eye(4,dtype='float64')
            T[:3,:3] =  mrob.quat_to_so3(quat)
            T[0, 3] = d[3]
            T[1, 3] = d[4]
            T[2, 3] = d[5]
            factors[int(d[1]),int(d[2])] = mrob.SE3(T).ln()

            # matrix information. g2o convention
            W = np.array(
                                  [[ d[10], d[11], d[12], d[13], d[14], d[15] ],
                                   [ d[11], d[16], d[17], d[18], d[19], d[20] ],
                                   [ d[12], d[17], d[21], d[22], d[23], d[24] ],
                                   [ d[13], d[18], d[22], d[25], d[26], d[27] ],
                                   [ d[14], d[19], d[23], d[26], d[28], d[29] ],
                                   [ d[15], d[20], d[24], d[27], d[29], d[30] ]], dtype='float64')
            # mrob convetion is however xi = [w, v], so we need to permute these values
            P = np.zeros((6,6))
            P[:3,3:] = np.eye(3)
            P[3:,:3] = np.eye(3)
            factor_inf[int(d[1]), int(d[2])] = P @ W @ P.transpose()
            factors_dictionary[int(d[2])].append(int(d[1]))
        else:
            # VERTEX_SE3:QUAT id[1], x[2],y[3],z[4],qx[5],qy[6],qz[7],qw[8]
            # these are the initial guesses for node states
            # transform to a RBT
            # transforming quaterntion to SE3
            quat = np.array([d[5],d[6],d[7],d[8]],dtype='float64')
            T = np.eye(4,dtype='float64')
            T[:3,:3] =  mrob.quat_to_so3(quat)
            T[0, 3] = d[2]
            T[1, 3] = d[3]
            T[2, 3] = d[4]
            vertex_ini[int(d[1])] = mrob.SE3(T).ln()
            # create an empty list of pairs of nodes (factor) connected to each node
            factors_dictionary[int(d[1])] = []

#print(factors_dictionary)

# Initialize FG
graph = mrob.FGraph(2200,8650)
x = vertex_ini[0]
print(x)
n = graph.add_node_pose_3d(x)
print('node 0 id = ', n) # id starts at 1
graph.add_factor_1pose_3d(x,n,1e9*np.identity(6))
processing_time = []

# start events, we solve for each node, adding it and it corresponding factors
# in total takes 0.3s to read all datastructure
for t in range(1,N):
    x = vertex_ini[t]
    n = graph.add_node_pose_3d(x)
    assert t == n, 'index on node is different from counter'

    # find factors to add. there must be 1 odom and other observations
    connecting_nodes = factors_dictionary[n]

    for nodeOrigin in factors_dictionary[n]:
        # inputs: obs, idOrigin, idTarget, invCov
        obs = factors[nodeOrigin, t]
        covInv = factor_inf[nodeOrigin, t]
        #graph.add_factor_2poses_3d(obs, nodeOrigin,t,covInv)
        # for end. no more loop inside the factors
        
        
    # solve the problem 7s 2500nodes
    start = time.time()
    graph.solve_batch()
    end = time.time()
    print('Iteration = ', t, ', chi2 = ', graph.chi2() , ', time on calculation [ms] = ', 1e3*(end - start))
    processing_time.append(1e3*(end - start))


    # plot the current problem
    if (t+1) % 100 == 0:
        print_3d_graph(graph)
        pass




graph.print(False)

    



