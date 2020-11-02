#
# add path to local library mrob on bash_aliases: "export PYTHONPATH=${PYTHONPATH}:${HOME}/mrob/mrob/lib"
import mrob
import numpy as np

# create graph
graph = mrob.fgraph.FGraph()

#initial point at [0,0,0] with some noise
#n1 = graph.add_node_pose_3d(mrob.geometry.SE3(np.random.randn(6)*0.05))

# non-initialized landmarks, but they could be initialiazed
l1 = graph.add_node_plane_4d(np.array([0,1,0,0]))
l2 = graph.add_node_plane_4d(np.array([1,0,0,0]))
l3 = graph.add_node_plane_4d(np.array([1,0,0,0]))
n1 = graph.add_node_pose_3d(mrob.geometry.SE3(np.array([0.0791869, 0.0458932,-0.0127219, -0.085251, 0.0202256, 0.0510333])))
print('node pose id = ', n1, ' , node landmark 1 id = ', l1 , ' , node landmark 2 id = ', l2, ' , node landmark 3 id = ', l3)


# anchor factor
W_0 = np.identity(6)
T_0 = mrob.geometry.SE3(np.array([0,0,0,10,20,0]))
#T_0 = mrob.geometry.SE3(np.random.randn(6)*0.05)
n2 = graph.add_node_pose_3d(mrob.geometry.SE3())
graph.add_factor_1pose_3d(T_0,n1,1e6*W_0)
W = np.identity(4)

# plane factors
# ground plane aligned with z
obs = np.array([0,0,1,3])
graph.add_factor_1pose_1plane_4d(obs,n1,l1,W)

#Planes should be a 4d vector normalized at  |n|=1. Constructor takes care of normalizing.
obs = np.array([1,0,0,1])
graph.add_factor_1pose_1plane_4d(obs,n1,l2,W)

# PLane facing Y
obs = np.array([0,-1,0.1,-15])
graph.add_factor_1pose_1plane_4d(obs,n1,l3,W)
#graph.print(True)



# plane factors at time 2
# ground plane aligned with z, stays the same
obs = np.array([0,0,1,3])
graph.add_factor_1pose_1plane_4d(obs,n2,l1,W)

obs = np.array([1,0,0,1])
graph.add_factor_1pose_1plane_4d(obs,n2,l2,W)

# PLane facing Y
obs = np.array([0,-1,0.1,-17])
graph.add_factor_1pose_1plane_4d(obs,n2,l3,W)
    #graph.print(True)



print('\n\n\n Solving Fgraph:\n')
#graph.solve(mrob.fgraph.GN) #1 iteration of Gauss-Newton
graph.solve(mrob.fgraph.LM) #as many iterations until convergence for Levenberg Marquardt
graph.print(True)
print('Current chi2 = ', graph.chi2() ) # re-evaluates the error, on print it is only the error on evalation before update


