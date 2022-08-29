#
# add path to local library mrob on bash_aliases: "export PYTHONPATH=${PYTHONPATH}:${HOME}/mrob/mrob/lib"
import mrob
import numpy as np

# create graph
graph = mrob.FGraph()

#initial point at [0,0,0] with some noise
n1 = graph.add_node_pose_2d(np.random.randn(3)*0.0005)
#n1 = graph.add_node_pose_2d(np.zeros(3)*0.0005)# hay un problem aqui cuando se inicia con el def constructor
W_0 = np.identity(3)
graph.add_factor_1pose_2d(np.zeros(3),n1,1e6*W_0)
#n1 = graph.add_node_pose_2d(np.zeros(3), mrob.NODE_ANCHOR)
# non-initialized landmarks, but they could be initialiazed when adding factors
# In case of 2d landmarks, they might be sensitive to IC, around 0 of initiakl pose and landmark On those cases might (rearely) GN fail to converge.
l1 = graph.add_node_landmark_2d(np.array([0,0]))
l2 = graph.add_node_landmark_2d(np.array([0,0]))
print('node pose id = ', n1, ' , node landmark 1 id = ', l1 , ' , node landmark 2 id = ', l2)


# landmark factors
obs = np.array([1,0])
W = np.identity(2)
graph.add_factor_1pose_1landmark_2d(obs,n1,l1,W)
obs = np.array([1,np.pi/2])
graph.add_factor_1pose_1landmark_2d(obs,n1,l2,W)



print('\n\n\n Solving Fgraph:\n')
graph.solve(mrob.GN) #1 iteration of Gauss-Newton
print(graph.get_information_matrix())
#graph.print(True)
#graph.solve(mrob.LM)
#graph.print(True)
print(graph.get_estimated_state())
print('Current chi2 = ', graph.chi2() ) # re-evaluates the error, on print it is only the error on evalation before update


