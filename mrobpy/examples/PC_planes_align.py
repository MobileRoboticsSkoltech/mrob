import mrob
import numpy as np
import open3d


def create_planes(labels, points):
	segments = []
	for l in np.unique(labels):
		if (l != -1):
			segments.append(np.array(points[labels == l]))
	return segments

def plot_segments(segments, color, T = []):
	pcds = []
	for i in segments:
		pcd = open3d.geometry.PointCloud()
		pcd.points = open3d.utility.Vector3dVector(i)
		pcd.paint_uniform_color(np.array(color).astype(np.float64))   
		if T!=[]:
			pcd.transform(T)
		pcds.append(pcd)
	return pcds

def draw_planes():
    pass
    #TODO, same methods to take Points thatn in crate points

# 1) Preprecess/Generate points each of them labeled with plane ID
# -----------------------------------------------------------------------------------
points = 1500
planes = 3
poses = 5

synthetic_points = mrob.CreatePoints(points,planes,poses, 0.001)
pcds = []
pcds_updated = []
for i in range(poses-1):
	labels = synthetic_points.get_point_plane_ids(i)
	points = np.array(synthetic_points.get_point_cloud(i))
	s = i/poses
	pcds.extend(plot_segments(create_planes(labels, points), color=[1-s,0,s]))

open3d.visualization.draw_geometries(pcds)

# 2) Generate structure
# -----------------------------------------------------------------------------------
problem = mrob.PlaneRegistration() #empty creator
# fills in all data from synthetic points into problem for optimiation
synthetic_points.create_plane_registration(problem)
problem.print()

# 3) Solve Plane aligment
# -----------------------------------------------------------------------------------
problem.solve()
problem.print()
