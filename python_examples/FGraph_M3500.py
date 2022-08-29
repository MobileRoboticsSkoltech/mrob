#
import mrob
import numpy as np
import time

import matplotlib.pyplot as plt



def print_2d_graph(graph):
    '''This function draws the state variables for a 2D pose graph'''
    
    # read graph, returns a list (vector) of state (np arrays)
    x = graph.get_estimated_state()
    prev_p = np.zeros(3)
    plt.figure()
    p = x[0]
    plt.plot(p[0],p[0],'ob') 
    for p in x:
        #plt.plot(p[0],p[1],'ob')
        plt.plot((prev_p[0],p[0]),(prev_p[1],p[1]) , '-b')
        prev_p = p
    plt.show()




# Initialize data structures
vertex_ini = {}
factors = {}
factors_dictionary = {}
N = 3500

# load file
with open('../benchmarks/M3500.txt', 'r') as file:
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

#print(factors_dictionary)

# Initialize FG
graph = mrob.FGraph()
x = np.zeros(3)
n = graph.add_node_pose_2d(x,mrob.NODE_ANCHOR)
print('node 0 id = ', n) # id starts at 1
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
        
        
    # solve the problem 7s 2500nodes
    start = time.time()
    #graph.solve(mrob.GN)
    end = time.time()
    #print('Iteration = ', t, ', chi2 = ', graph.chi2() , ', time on calculation [ms] = ', 1e3*(end - start))
    processing_time.append(1e3*(end - start))
    
    # problem error or xi2
    #print('X2', t)

    # plot the current problem
    if (t+1) % 500 == 0:
        #print_2d_graph(graph)
        pass

print('Initial problem drawn')
print_2d_graph(graph)
print('current initial chi2 = ', graph.chi2() )
start = time.time()
graph.solve(mrob.LM, 50)
end = time.time()
print('\nLM chi2 = ', graph.chi2() , ', total time on calculation [s] = ', 1e0*(end - start))
print('solution drawn')
print_2d_graph(graph)


if 0:
    # Information matrix
    import matplotlib.pyplot as plt
    L = graph.get_information_matrix()
    plt.spy(L, marker='o', markersize=5)
    plt.title('Information matrix $\Lambda$')
    plt.show()
    
    # ch2
    chi2_array = graph.get_chi2_array()
    plt.plot(chi2_array)
    plt.show()

# alternative use Gauss-Newton
if 0:
    graph.solve(mrob.fgraph.GN)
    print('Iter 0 chi2 = ', graph.chi2() )
    graph.solve(mrob.GN)
    print('Iter 1 chi2 = ', graph.chi2() )
    graph.solve(mrob.GN)
    print('Iter 2 chi2 = ', graph.chi2() )
    graph.solve(mrob.GN)
    print('Iter 3 chi2 = ', graph.chi2() )
    print_2d_graph(graph)

    graph.solve(mrob.GN)
    print('Iter 4 chi2 = ', graph.chi2() )
    graph.solve(mrob.GN)
    print('Iter 5 chi2 = ', graph.chi2() ) #already converges
    print_2d_graph(graph)



