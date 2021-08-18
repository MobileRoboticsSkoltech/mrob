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

N_points = 5
N_planes = 2
N_poses = 2

synthetic = mrob.registration.CreatePoints(N_points,N_planes,N_poses, 0.05, 0.1) #point noise, bias noise
T_gt = synthetic.get_ground_truth_last_pose()
T_gt.print()
#draw_planes(synthetic, synthetic.get_trajectory())

graph = mrob.FGraph()

for t in range(N_planes):
    ef1 = graph.add_eigen_factor_plane()
    # It is an ordered progression 0:N-1, no need for dict
    print('EFactor id = ', ef1)

for t in range(N_poses):
    n1 = graph.add_node_pose_3d(mrob.geometry.SE3())
    # It is an ordered progression 0:N-1, no need for dict
    print('Pose node id = ', n1)

for t in range(N_poses):
    print('Processing pose ', t)
    points = synthetic.get_point_cloud(t)
    indexes = synthetic.get_point_plane_ids(t)
    for p,i in zip(points,indexes):
        print('point ', p, 'index ', i)
        graph.eigen_factor_plane_add_point(planeEigenId = i,
                                   nodePoseId = t,
                                   point = p,
                                   W = 1.0)

graph.solve(mrob.LM)
graph.print(True)
