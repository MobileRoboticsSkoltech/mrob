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
    prev_p = x[0]
    for xi in x:
        Ti = mrob.SE3(xi)
        p = Ti.T()[:3,3]
        #print(Ti.T())
        ax.plot((prev_p[0],p[0]),(prev_p[1],p[1]),(prev_p[2],p[2]) , '-b')
        prev_p = np.copy(p)
    plt.show()



def plot_from_vertex(vertex):
    "given a dictionary of vertex plots the xyz"
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    prev_p = vertex[0]
    for i in range(1,N):
        p = vertex[i]
        #ax.scatter(p[0],p[1],p[2],color='green')
        ax.plot((prev_p[0],p[0]),(prev_p[1],p[1]),(prev_p[2],p[2]) , '-b')
        prev_p = np.copy(p)
    plt.show()

# Initialize data structures
#vertex_ini = {}
factors = {}
factor_inf = {}
factors_dictionary = {}
N = 2500

for i in range(N):
    # create an empty list of pairs of nodes (factor) connected to each node
    factors_dictionary[i] = []

# sphere2500 from gtsam, using the TORO format
#with open('../../datasets/sphere2500.txt', 'r') as file:
with open('../../datasets/sphere2500_groundtruth.txt', 'r') as file:
    for line in file:
        d = line.split()
        # read edges and vertex
        if d[0] == 'EDGE3':
            # EDGE3 id_origin[1]  id_target[2]   x[3],y[4],z[5],roll[6],pithc[7],yaw[8]
            #     L11[9], L12[10], L13, L14, L15, L16
            #              L22[15], L23, L24, L25, L26
            #                       L33[20], L34, L35, L36
            #                                L44[24], L45 L46
            #                                       L55[27] L56
            #                                             L66[29]

            # transforming quaterntion to SE3
            rpy = np.array([d[6],d[7],d[8]],dtype='float64')
            T = np.eye(4,dtype='float64')
            T[:3,:3] =  mrob.rpy_to_so3(rpy)
            T[0, 3] = d[3]
            T[1, 3] = d[4]
            T[2, 3] = d[5]
            print('ds: ', d[1], d[2], d[3],d[4],d[5],d[6],d[7],d[8])
            print(T)
            factors[int(d[1]),int(d[2])] = mrob.SE3(T).ln()

            #test we are transforming correctly
            #Tres = mrob.SE3(factors[int(d[1]),int(d[2])])
            #print(Tres.distance(mrob.SE3(T)))

            # matrix information. g2o and TORO convention
            W = np.array(
                                  [[  d[9], d[10], d[11], d[12], d[13], d[14] ],
                                   [ d[10], d[15], d[16], d[17], d[18], d[19] ],
                                   [ d[11], d[16], d[20], d[21], d[22], d[23] ],
                                   [ d[12], d[17], d[21], d[24], d[25], d[26] ],
                                   [ d[13], d[18], d[22], d[25], d[27], d[28] ],
                                   [ d[14], d[19], d[23], d[26], d[28], d[29] ]], dtype='float64')
            # mrob convetion is however xi = [w, v], so we need to permute these values
            P = np.zeros((6,6))
            P[:3,3:] = np.eye(3)
            P[3:,:3] = np.eye(3)
            factor_inf[int(d[1]), int(d[2])] = P @ W @ P.transpose()
            factors_dictionary[int(d[2])].append(int(d[1]))


# Initialize FG
graph = mrob.FGraph(2500,4500)
x = np.zeros(6)
n = graph.add_node_pose_3d(x)
W = np.eye(6)
W[3:, 3:] = np.eye(3) * 100
graph.add_factor_1pose_3d(x,n,1e5*W)
processing_time = []

# start events, we solve for each node, adding it and it corresponding factors
# in total takes 0.3s to read all datastructure
for t in range(1,N):
    #x = vertex_ini[t] # on sphere no initial value, initialized to 0s
    n = graph.add_node_pose_3d(x)
    assert t == n, 'index on node is different from counter'

    # find factors to add. there must be 1 odom and other observations
    connecting_nodes = factors_dictionary[n]

    for nodeOrigin in factors_dictionary[n]:
        # inputs: obs, idOrigin, idTarget, invCov
        obs = factors[nodeOrigin, t]
        covInv = factor_inf[nodeOrigin, t]
        if t - nodeOrigin == 1:
            graph.add_factor_2poses_3d(obs, nodeOrigin,t,covInv,True)
        else:
            graph.add_factor_2poses_3d(obs, nodeOrigin,t,covInv)
        # for end. no more loop inside the factors
        
        
    # solve the problem 7s 2500nodes
    start = time.time()
    graph.solve_batch()
    end = time.time()
    #print('Iteration = ', t, ', chi2 = ', graph.chi2() , ', time on calculation [ms] = ', 1e3*(end - start))
    processing_time.append(1e3*(end - start))


    # plot the current problem
    if (t+1) % 2500 == 0:
        #graph.print(True)
        print_3d_graph(graph)
        pass





#graph.solve_batch()
#print('chi2 = ', graph.chi2())
#graph.solve_batch()
print('chi2 = ', graph.chi2())
#graph.print(True)

# testing that transformation are correctly handled by our preprocessing
if 0:
    vertex_gt = []
    vertex_gt_xyz = []
    vertex_gt.append(np.eye(4))
    vertex_gt_xyz.append(np.zeros(3))
    for t in range(1, N):
        # find odom factors
        connecting_nodes = factors_dictionary[t]

        print(connecting_nodes, '\n and current node index= ', t)
        for nodeOrigin in factors_dictionary[t]:
            if t - nodeOrigin == 1:
                obs = factors[nodeOrigin, t]
                print('size of = ', len(vertex_gt), '\n and node: ', nodeOrigin)
                xtp = mrob.SE3(obs)
                xt = vertex_gt[nodeOrigin] @ xtp.T()
                xtp.print()
                print(vertex_gt[nodeOrigin])
                print(xt)
                vertex_gt.append(xt)
                vertex_gt_xyz.append(xt[:3,3])

    print(vertex_gt_xyz)
    plot_from_vertex(vertex_gt_xyz)

    



