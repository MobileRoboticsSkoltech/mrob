#
# add path to local library mrob on bashr_rc: "export PYTHONPATH=${PYTHONPATH}:${HOME}/mrob/mrob/build/mrobpy"
import mrob
import numpy as np

# example translated from FGrpah/examples/example_solver_3d_landmarks.cpp

graph = mrob.FGraph(mrob.QUADRATIC)

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
#graph.print(True)




# testing matrix
import matplotlib.pyplot as plt
L = graph.get_information_matrix()
plt.spy(L, marker='o', markersize=5)
plt.title('Information matrix $\Lambda$')
plt.show()
