#
# add path to local library mrob on bashr_rc: "export PYTHONPATH=${PYTHONPATH}:${HOME}/mrob/mrob/lib"
import mrob
import numpy as np
import open3d
import time


# generate random data
N_points = 500
N_planes = 3
N_poses = 2
point_noise = 0.05


synthetic_points = mrob.registration.CreatePoints(N_points,N_planes,N_poses, point_noise,0.1)
X = synthetic_points.get_point_cloud(1)
ids =  synthetic_points.get_point_plane_ids(1)

T = mrob.geometry.SE3(np.random.randn(6))
T.print()
Xa = np.array(X)
Y = T.transform_array(X) + np.random.randn(N_points,3)*0.1

def pcd_1(X, color, T = []):
	pcd = open3d.geometry.PointCloud()
	pcd.points = open3d.utility.Vector3dVector(X)
	if T!=[]:
	    pcd.transform(T)
	pcd.paint_uniform_color(color)
	return pcd

def vis_her(X, Y, T = np.identity(4)):
    blue = np.array([0,0,1], dtype='float64')
    red = np.array([1,0,0], dtype='float64')
    open3d.visualization.draw_geometries([pcd_1(X,red,T), pcd_1(Y,blue)])

def vis_arr(X):
	pcd = open3d.PointCloud()
	pcd.points =open3d.utility.Vector3dVector(X)
	pcd.paint_uniform_color(np.random.rand(3,).astype(np.float64))
	open3d.visualization.draw_geometries([pcd])


# initial visualization
vis_her(X,Y)

# calculate planes
pi = []
centroids = []
for p in range(N_planes):
    #indexes of the points corresponding to the plane
    Yi = [Y[i] for i in range(N_points) if ids[i] == p]
    Yi = np.array(Yi)
    pi.append( mrob.registration.estimate_normal(Yi))
    centroids.append(mrob.registration.estimate_centroid(Yi))
print(centroids)
# solve the problem. It finds transformation from the point reference (Y) frame to the plane reference frame (X)
graph = mrob.FGraph()
W = np.array([1])
print(W)
n1 = graph.add_node_pose_3d(mrob.geometry.SE3())
for i in range(N_points):
    graph.add_factor_1pose_point2plane(z_point_x = X[i],
                                       z_point_y = centroids[ids[i]], #Y[i],
                                       z_normal_y = pi[ids[i]],
                                       nodePoseId = n1,
                                       obsInf = W)

# Solve the problem for the potint to point
graph_p2p = mrob.FGraph()
W = np.identity(3)
print('W point2point = ', W)
n1 = graph_p2p.add_node_pose_3d(mrob.geometry.SE3())
for i in range(N_points):
    graph_p2p.add_factor_1pose_point2point(z_point_x = X[i],
                                       z_point_y = centroids[ids[i]], #Y[i],
                                       nodePoseId = n1,
                                       obsInf = W)


#graph.print(True)
print('Current state of the graph: chi2 = ' , graph.chi2() )
start = time.time()
#XXX initial ideal lambda is around 1e-1, vs the default 1e-5, so many more iterations are needed.
graph.solve(mrob.LM, 30)
end = time.time()
print(', chi2 = ', graph.chi2() , ', time on calculation [s] = ', 1e0*(end - start))
results = graph.get_estimated_state()

print(results)
Testimated = mrob.geometry.SE3(results[0])
# initial visualization
print('Error in poses rotation= ', T.distance_rotation(Testimated), ', distance trans = ', T.distance_trans(Testimated))
vis_her(X,Y,Testimated.T())


print('Current state of the graph point2point: chi2 = ' , graph_p2p.chi2() )
start = time.time()
#XXX initial ideal lambda is around 1e-1, vs the default 1e-5, so many more iterations are needed.
graph_p2p.solve(mrob.LM, 30)
end = time.time()
print(', chi2 = ', graph_p2p.chi2() , ', time on calculation [s] = ', 1e0*(end - start))
results = graph_p2p.get_estimated_state()

print(results)
Testimated = mrob.geometry.SE3(results[0])
# initial visualization
print('Error in poses rotation= ', T.distance_rotation(Testimated), ', distance trans = ', T.distance_trans(Testimated))
vis_her(X,Y,Testimated.T())






#L = graph.get_information_matrix().todense()
#D, U = np.linalg.eig(L)
#print(D)
#print(U)


