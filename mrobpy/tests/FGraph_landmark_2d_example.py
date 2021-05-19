#
# add path to local library mrob on bash_aliases: "export PYTHONPATH=${PYTHONPATH}:${HOME}/mrob/mrob/lib"
import mrob
import numpy as np

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


