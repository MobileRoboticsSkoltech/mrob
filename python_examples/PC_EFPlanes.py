import mrob
import numpy as np
import open3d


def draw_planes(synthetic,traj=[]):
    pcds = []
    for i in range(synthetic.get_number_poses()):
        pc = open3d.geometry.PointCloud()
        pc.points = open3d.utility.Vector3dVector(np.array(synthetic.get_point_cloud(i)))
        s = i/poses
        pc.paint_uniform_color([0.5,1-s, (1-s)/2])
        if traj!=[]:
            if type(traj[i]) == mrob.geometry.SE3:
                pc.transform(traj[i].T()) #traj is a list of SE3
            else:
                pc.transform(traj[i]) #traj is a list of 4x4 np arrays
        pcds.append(pc)
    open3d.visualization.draw_geometries(pcds)

points = 2000
planes = 4
poses = 3

synthetic_points = mrob.registration.CreatePoints(points,planes,poses, 0.05, 0.1) #point noise, bias noise
T_gt = synthetic_points.get_ground_truth_last_pose()
T_gt.print()
draw_planes(synthetic_points, synthetic_points.get_trajectory())

graph = mrob.FGraph()

ef1 = graph.add_eigen_factor_plane()
print(ef1)
n1 = graph.add_node_pose_3d(mrob.geometry.SE3())
p = np.array([1,2,-3])
graph.eigen_factor_plane_add_point(planeEigenId = ef1,
                                   nodePoseId = n1,
                                   point = p,
                                   W = 1.0)

graph.print(True)
