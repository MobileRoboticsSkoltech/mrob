import numpy as np
import mrob
import time

from os import path

import pytest

class TestFGraph:
    def test_2d(self):
        # example similar to ./FGrpah/examples/example_FGraph_solve.cpp

        # create graph, reserving for 10 nodes and 20 factors if indicated
        graph = mrob.FGraph()

        # TODO give more nodes, such as a rectangle.
        x = np.random.randn(3)
        n1 = graph.add_node_pose_2d(x)
        x = 1 + np.random.randn(3)*1e-1
        n2 = graph.add_node_pose_2d(x)
        print('node 1 id = ', n1, ' , node 2 id = ', n2)


        invCov = np.identity(3)
        graph.add_factor_1pose_2d(np.zeros(3),n1,1e6*invCov)
        graph.add_factor_2poses_2d(np.ones(3),n1,n2,invCov)

        graph.solve(mrob.GN)
        graph.print(True)

    def test_landmark_2d(self):
        # create graph
        graph = mrob.FGraph()

        #initial point at [0,0,0] with some noise
        n1 = graph.add_node_pose_2d(np.random.randn(3)*0.05)
        # non-initialized landmarks, but they could be initialiazed
        l1 = graph.add_node_landmark_2d(np.array([0,0]))
        l2 = graph.add_node_landmark_2d(np.zeros(2))
        print('node pose id = ', n1, ' , node landmark 1 id = ', l1 , ' , node landmark 2 id = ', l2)

        # anchor factor
        W_0 = np.identity(3)
        graph.add_factor_1pose_2d(np.zeros(3),n1,1e6*W_0)

        # landmark factors
        obs = np.array([1,0])
        W = np.identity(2)
        graph.add_factor_1pose_1landmark_2d(obs,n1,l1,W)
        obs = np.array([1,np.pi/2])
        graph.add_factor_1pose_1landmark_2d(obs,n1,l2,W)
        graph.print(True)



        print('\n\n\n Solving Fgraph:\n')
        #graph.solve(mrob.GN) #1 iteration of Gauss-Newton
        graph.solve(mrob.LM) #as many iterations until convergence for Levenberg Marquardt
        graph.print(True)
        print('Current chi2 = ', graph.chi2() ) # re-evaluates the error, on print it is only the error on evalation before update


    def test_landmark_3d(self):
        # example translated from FGrpah/examples/example_solver_3d_landmarks.cpp

        graph = mrob.FGraph()

        n1 = graph.add_node_pose_3d(mrob.geometry.SE3(np.random.randn(6)*0.05))
        # non-initialized landmarks, nut they could be initialiazed
        l1 = graph.add_node_landmark_3d(np.zeros(3))
        l2 = graph.add_node_landmark_3d(np.zeros(3))
        l3 = graph.add_node_landmark_3d(np.zeros(3))
        print('node pose id = ', n1, ' , node landmark 1 id = ', l1 , ' , node landmark 2 id = ', l2, ' , node landmark 3 id = ', l3)

        # anchor factor
        W_0 = np.identity(6)
        graph.add_factor_1pose_3d(mrob.geometry.SE3(),n1,1e6*W_0)

        # landmark factors
        obs = np.array([1,0,0])
        W = np.identity(3)
        # XXX there is a flag (not implemented) for autoinitializaing landmarks (with inverse function). Please tell me if it is necessary/useful
        graph.add_factor_1pose_1landmark_3d(obs,n1,l1,W)
        obs = np.array([1,1,0])
        graph.add_factor_1pose_1landmark_3d(obs,n1,l2,W)
        obs = np.array([1,1,1])
        graph.add_factor_1pose_1landmark_3d(obs,n1,l3,W)

        graph.print(True)



        print('\n\n\n Solving Fgraph:\n')
        #graph.solve(mrob.fgraph.GN) #1 iteration of Gauss-Newton
        graph.solve(mrob.LM) #as many iterations until convergence for Levenberg Marquardt
        graph.print(True)

    def test_m3500(self):
        # Initialize data structures
        vertex_ini = {}
        factors = {}
        factors_dictionary = {}
        N = 3500

        # load file
        with open(path.join(path.dirname(__file__), '../../benchmarks/M3500.txt'), 'r') as file:
            for line in file:
                d = line.split()
                # read edges and vertex, in TORO format
                if d[0] == 'EDGE2':
                    # EDGE2 id_origin   id_target   dx   dy   dth   I11   I12  I22  I33  I13  I23
                    factors[int(d[1]),int(d[2])] = np.array([d[3], d[4], d[5], d[6],d[7],d[8],d[9],d[10],d[11]],dtype='float64')
                    factors_dictionary[int(d[2])].append(int(d[1]))
                else:
                    # VERTEX2 id x y theta
                    # these are the initial guesses for node states
                    vertex_ini[int(d[1])] = np.array([d[2], d[3], d[4]],dtype='float64')
                    # create an empty list of pairs of nodes (factor) connected to each node
                    factors_dictionary[int(d[1])] = []


        # Initialize FG
        graph = mrob.FGraph()
        x = np.zeros(3)
        n = graph.add_node_pose_2d(x)
        print('node 0 id = ', n) # id starts at 1
        graph.add_factor_1pose_2d(x,n,1e9*np.identity(3))
        processing_time = []

        # start events, we solve for each node, adding it and it corresponding factors
        # in total takes 0.3s to read all datastructure
        for t in range(1,N):
            x = vertex_ini[t]
            n = graph.add_node_pose_2d(x)
            assert t == n, 'index on node is different from counter'

            # find factors to add. there must be 1 odom and other observations
            connecting_nodes = factors_dictionary[n]

            for nodeOrigin in factors_dictionary[n]:
                # inputs: obs, idOrigin, idTarget, invCov
                obs = factors[nodeOrigin, t][:3]
                covInv = np.zeros((3,3))
                # on M3500 always diagonal information matrices
                covInv[0,0] = factors[nodeOrigin, t][3]
                covInv[1,1] = factors[nodeOrigin, t][5]
                covInv[2,2] = factors[nodeOrigin, t][6]
                graph.add_factor_2poses_2d(obs, nodeOrigin,t,covInv)
                # for end. no more loop inside the factors
                
                
            # solve the problem iteratively 2500nodes
            #graph.solve(mrob.GN)

        print('current initial chi2 = ', graph.chi2() )
        start = time.time()
        graph.solve(mrob.LM, 50)
        end = time.time()
        print('\nLM chi2 = ', graph.chi2() , ', total time on calculation [s] = ', 1e0*(end - start))

    def test_sphere(self):
        # Initialize data structures
        vertex_ini = {}
        factors = {}
        factor_inf = {}
        factors_dictionary = {}
        N = 2500


        # load file, .g2o format from https://github.com/RainerKuemmerle/g2o/wiki/File-Format
        #file_path = '../../benchmarks/sphere_gt.g2o'
        file_path = '../../benchmarks/sphere.g2o'
        with open(path.join(path.dirname(__file__), file_path), 'r') as file:
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


