#
import mrob
import numpy as np

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
graph.add_factor_1pose_2d(np.array([0,0,np.pi/4]),n1,1e6*invCov)
graph.add_factor_2poses_2d(np.ones(3),n1,n2,invCov)

graph.solve(mrob.LM)
graph.print(True)


