#
import mrob
import numpy as np
import time

# Initialize data structures
vertex_ini = {}
factors = {}
factor_inf = {}
factors_dictionary = {}
N = 2500


# load file, .g2o format from https://github.com/RainerKuemmerle/g2o/wiki/File-Format
#file_path = '../../benchmarks/sphere_gt.g2o'
file_path = '../../benchmarks/sphere.g2o'
with open(file_path, 'r') as file:
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
            T[:3,:3] =  mrob.geometry.quat_to_so3(quat)
            T[0, 3] = d[3]
            T[1, 3] = d[4]
            T[2, 3] = d[5]
            factors[int(d[1]),int(d[2])] = mrob.geometry.SE3(T)

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
            #print(W)
            #print(factor_inf[int(d[1]), int(d[2])])
            factors_dictionary[int(d[2])].append(int(d[1]))
        else:
            # VERTEX_SE3:QUAT id[1], x[2],y[3],z[4],qx[5],qy[6],qz[7],qw[8]
            # these are the initial guesses for node states
            # transform to a RBT
            # transforming quaterntion to SE3
            quat = np.array([d[5],d[6],d[7],d[8]],dtype='float64')
            T = np.eye(4,dtype='float64')
            T[:3,:3] =  mrob.geometry.quat_to_so3(quat)
            T[0, 3] = d[2]
            T[1, 3] = d[3]
            T[2, 3] = d[4]
            #print('ds: ', d[1], d[2], d[3],d[4],d[5],d[6],d[7],d[8])
            #print(T)
            vertex_ini[int(d[1])] = mrob.geometry.SE3(T)
            # create an empty list of pairs of nodes (factor) connected to each node
            factors_dictionary[int(d[1])] = []

#print(factors_dictionary)


# Initialize FG
graph = mrob.FGraph()
x = vertex_ini[0]
print(x.T())
n = graph.add_node_pose_3d(x)
W = np.eye(6)
graph.add_factor_1pose_3d(x,n,1e5*W)
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
        #covInv = factor_inf[nodeOrigin, t]
        covInv = np.eye(6)
        #covInv[3:,3:] = np.eye(3)*100
        graph.add_factor_2poses_3d(obs, nodeOrigin,t,covInv)
        # for end. no more loop inside the factors
        
        
    # solve the problem incrementally
    #graph.solve(mrob.GN)
    #graph.solve(mrob.LM, 3)        


# Solves the batch problem
print('Current state of the graph: chi2 = ' , graph.chi2() )
start = time.time()
graph.solve(mrob.LM,8)
end = time.time()
print(', chi2 = ', graph.chi2() , ', time on calculation [s] = ', 1e0*(end - start))


