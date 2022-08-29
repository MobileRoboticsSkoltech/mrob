#
import mrob
import numpy as np

# this example is to show the effect of anchors: We fix some given nodes, that are excluded from optimization updates

print('Anchor on first node:')
graph = mrob.FGraph()

x = np.random.randn(3)
n1 = graph.add_node_pose_2d(x,mrob.NODE_ANCHOR)
x = 1 + np.random.randn(3)*1e-1
n2 = graph.add_node_pose_2d(x)
print('node 1 id = ', n1, ' , node 2 id = ', n2)


invCov = np.identity(3)
graph.add_factor_2poses_2d(np.ones(3),n1,n2,invCov)

graph.solve(mrob.LM)
graph.print(True)

A = graph.get_adjacency_matrix()
A = A.todense()
print(A)


print('Anchor on second node:')
graph = mrob.FGraph()

x = np.random.randn(3)
n1 = graph.add_node_pose_2d(x)
x = 1 + np.random.randn(3)*1e-1
n2 = graph.add_node_pose_2d(x,mrob.NODE_ANCHOR)
print('node 1 id = ', n1, ' , node 2 id = ', n2)


invCov = np.identity(3)
graph.add_factor_2poses_2d(np.ones(3),n1,n2,invCov)

graph.solve(mrob.LM)
graph.print(True)

A = graph.get_adjacency_matrix()
A = A.todense()
print(A)


print('virtual anchor by a high cov:')
graph = mrob.FGraph()

# TODO give more nodes, such as a rectangle.
x = np.random.randn(3)
n1 = graph.add_node_pose_2d(x)
invCov = np.identity(3)
graph.add_factor_1pose_2d(x,n1,1e6*invCov)
x = 1 + np.random.randn(3)*1e-1
n2 = graph.add_node_pose_2d(x)
print('node 1 id = ', n1, ' , node 2 id = ', n2)
graph.add_factor_2poses_2d(np.ones(3),n1,n2,invCov)

graph.solve(mrob.LM)
graph.print(True)

A = graph.get_adjacency_matrix()
A = A.todense()
print(A)
