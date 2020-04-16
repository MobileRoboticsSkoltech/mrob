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

points = 1500
planes = 3
poses = 5

f_tr = mrob.CreatePoints(points,planes,poses, 0.001)
pcds = []
pcds_updated = []
for i in range(poses-1):
	labels = f_tr.get_point_plane_ids(i)
	points = np.array(f_tr.get_point_cloud(i))
	#points = f_tr.get_point_cloud(i)
	#print(type(points))
	points_next = np.array(f_tr.get_point_cloud(i+1))
	s = i/poses
	pcds.extend(plot_segments(create_planes(labels, points), color=[1-s,0,s]))
	T_arun = mrob.align_arun(points,points_next)
	labels = f_tr.get_point_plane_ids(i+1)
	pcds_updated.extend(plot_segments(create_planes(labels, points_next), color=[1-s,0,s], T = T_arun.T()))

open3d.visualization.draw_geometries(pcds)
open3d.visualization.draw_geometries(pcds_updated)
